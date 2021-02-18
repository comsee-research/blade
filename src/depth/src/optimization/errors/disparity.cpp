#include "disparity.h"

#include <pleno/unused.h>
#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

#include <pleno/processing/imgproc/improcess.h>

#define ENABLE_DEBUG_DISPLAY 0

//******************************************************************************
//******************************************************************************
//******************************************************************************
template <bool useBlur>
DisparityCostError_<useBlur>::DisparityCostError_(
	const Image& img_i_, const Image& img_j_, 
	const MicroImage& mi_i_, const MicroImage& mi_j_, 
	const PlenopticCamera& mfpc_, 
	P2D at_, BlurMethod method_
) : img_i{img_i_.clone()}, img_j{img_j_.clone()}, 
	mi_i{mi_i_}, mi_j{mi_j_}, mfpc{mfpc_},
	at{at_}, method{method_}
{ }

template <bool useBlur>
DisparityCostError_<useBlur>::DisparityCostError_(
	const DisparityCostError_& o
) : img_i{o.img_i.clone()}, img_j{o.img_j.clone()}, 
	mi_i{o.mi_i}, mi_j{o.mi_j}, mfpc{o.mfpc},
	at{o.at}, method{o.method}
{ } 

template <bool useBlur>
DisparityCostError_<useBlur>::DisparityCostError_(
	DisparityCostError_&& o
) : img_i{std::move(o.img_i)}, img_j{std::move(o.img_j)}, 
	mi_i{o.mi_i}, mi_j{o.mi_j}, mfpc{o.mfpc},
	at{std::move(o.at)}, method{o.method}
{ }

template <bool useBlur>
double DisparityCostError_<useBlur>::weight(double v) const 
{ 
	const P2D mli = mfpc.mla().nodeInWorld(mi_i.k, mi_i.l).head(2);
	const P2D mlj = mfpc.mla().nodeInWorld(mi_j.k, mi_j.l).head(2);
	
	const P2D deltaC = (mli - mlj) / mfpc.sensor().scale(); 
	P2D disparity = deltaC / v; 
	
	const double disp 	= disparity.norm();
	const double w 		= (v * v) / (disp * disp); 
	
	return w; 
}

template <bool useBlur>
bool DisparityCostError_<useBlur>::compute_at_pixel() const
{
	return not (at[0] == -1. and at[1] == -1.);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template <bool useBlur>
bool DisparityCostError_<useBlur>::operator()(
    const VirtualDepth& depth,
	ErrorType& error
) const
{    
	constexpr int window_size = 3;
    constexpr double threshold_reprojected_pixel = double(window_size * window_size) - 1.;
    constexpr double epsilon = 2.2204e-16;

    error.setZero();
    
	const double v = depth.v;
	const double radius = mfpc.mia().radius() - mfpc.mia().border();
    
//0) Check hypotheses:
	//0.1) Discard observation?
	const P2D mli = mfpc.mla().nodeInWorld(mi_i.k, mi_i.l).head(2);
	const P2D mlj = mfpc.mla().nodeInWorld(mi_j.k, mi_j.l).head(2);
	
	const P2D deltaC = (mli - mlj) / mfpc.sensor().scale(); 
	P2D disparity = deltaC / v; 
	
	if (disparity.norm() >= 2. * radius)
	{
		//PRINT_ERR("Discard observation for hypothesis = " << v << ", disparity = " << disparity.norm());
		return false;
	}
	//0.2) Is disparity estimation possible? Is object real?
	if (std::fabs(v) < 2. or mfpc.v2obj(v) < mfpc.focal()) 
	{
		error[0] = 255. / v;
		return true;
	}
    
//1) get ref and edi
	Image fref, ftarget;	
	img_i.convertTo(fref, CV_64FC1, 1./255.); //(i)-view is ref
	img_j.convertTo(ftarget, CV_64FC1, 1./255.); //(j)-view has to be warped

//2) compute mask	
	Image fmask = Image{fref.size(), CV_64FC1, cv::Scalar::all(1.)};
	trim_double(fmask, radius);
		
//3) warp image according to depth hypothesis
	//3.1) compute transformation
	cv::Mat M = (cv::Mat_<double>(2,3) << 1., 0., disparity[0], 0., 1., disparity[1]); //Affine transformation
	
	//3.2) warp mask and target	
	const auto interp = cv::INTER_LINEAR; //cv::INTER_CUBIC; //
	Image wmask, wtarget;
	cv::warpAffine(fmask, wmask, M, fmask.size(), interp + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	cv::warpAffine(ftarget, wtarget, M, ftarget.size(), interp + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	
	trim_double(wmask, radius); //Final mask =  micro-image mask INTER warped masked
	
	Image fedi, lpedi;	
	if constexpr (useBlur)
	{
		//3.3) compute blur	
		if (mi_i.type != mi_j.type) //not same type
		{
			const double A = mfpc.mlaperture(); //mm
			const double d = mfpc.d();			//mm
			const double ai = mfpc.obj2mla(mfpc.focal_plane(mi_i.type));  //mm, negative signed distance to mla
			const double aj = mfpc.obj2mla(mfpc.focal_plane(mi_j.type));  //mm, negative signed distance to mla

			const double mij = ((A * A * d) / 2.) * ((1. / ai) - (1. / aj)); //mm²
			const double cij = ((A * A * d * d) / 4.) * ((1. / (ai * ai)) - (1. / (aj * aj))); //mm²

			const double rel_blur = mij * (1. / v) + cij; //mm²	

			const double rho_sqr_r = rel_blur / (mfpc.sensor().scale() * mfpc.sensor().scale()); //pix²
			const double kappa = mfpc.params().kappa; 
			const double sigma_sqr_r = kappa * rho_sqr_r; //pix²
			const double sigma_r = std::sqrt(std::fabs(sigma_sqr_r)); //pix

			const bool isOrdered = (rel_blur >= 0.); //(i)-view is more defocused the the (j)-view
		
			if (isOrdered) //(i)-view is more defocused the the (j)-view
			{
		#if ENABLE_DEBUG_DISPLAY
				PRINT_DEBUG("Views: (edi = target); ref ("<< mi_i.type+1<<"), target ("<< mi_j.type+1<<")");
		#endif 
				fedi = wtarget; //(j)-view has to be equally-defocused
			}
			else //(j)-view is more defocused the the (i)-view
			{
		#if ENABLE_DEBUG_DISPLAY
				PRINT_DEBUG("Views: (edi = ref); ref ("<< mi_i.type+1<<"), target ("<< mi_j.type+1<<")");
		#endif 
				fedi = fref; //(i)-view has to be equally-defocused
			}
			
			switch (method)
			{	
				case S_TRANSFORM:
				{
					cv::Laplacian(fedi, lpedi, CV_64FC1); //FIXME: Kernel size ? 3x3
					cv::add(fedi, (std::fabs(sigma_sqr_r) / 4.) * lpedi, fedi);
					break;
				}	
				case GAUSSIAN_BLUR:
				{
					Image medi, bedi, bmask;
					
					medi = fedi.mul(wmask); //apply mask on edi
					cv::GaussianBlur(wmask, bmask, cv::Size{0,0}, sigma_r, sigma_r); //blur mask
					cv::GaussianBlur(fedi, bedi, cv::Size{0,0}, sigma_r, sigma_r); //blur masked image	
					cv::divide(bedi, bmask, fedi); //divide the blurred masked image by the blurred mask 
					break;
				}
				case APPROX_GAUSSIAN_BLUR: //blur w/o considering neighbors impact
				{
					cv::GaussianBlur(fedi, fedi, cv::Size{0,0}, sigma_r, sigma_r);
					break;
				}	
			}
		}
	}
	else
	{
		UNUSED(fedi); UNUSED(lpedi);
	}
	
	//3.4) apply mask
	Image finalref = fref.mul(wmask);
	Image finaltarget = wtarget.mul(wmask); 	
	
	//3.5) get sub window if at specific pixel
	const int h = img_i.rows;
	const int w = img_i.cols;
	
	cv::Rect window{0,0, w, h};
	if (compute_at_pixel())
	{
		double cu = 0., cv = 0.;
		
		if ((at - mi_i.center).norm() < (at-mi_j.center).norm()) {
			cu = mi_i.center[0]; cv = mi_i.center[1];
		} else {
			cu = mi_j.center[0]; cv = mi_j.center[1];
		}
		
		int s = static_cast<int>(std::round(at[0] - cu + double(w / 2.)));
		int t = static_cast<int>(std::round(at[1] - cv + double(h / 2.)));
		
		//FIXME: think about getting a rotated rectangle along EPI
		window = cv::Rect{s, t, window_size, window_size};	//FIXME: check bounds
	#if 0
		DEBUG_VAR(at);
		DEBUG_VAR(cu); DEBUG_VAR(cv);
		DEBUG_VAR(s); DEBUG_VAR(t);	
		DEBUG_VAR(window);
	#endif
	}
	
//4) compute cost	
	const double summask = cv::sum(wmask(window))[0];
	if (summask < threshold_reprojected_pixel) return false;
	
	const double normalization = 1. / (summask + epsilon); //number of pixel to take into account
	const double err = cv::norm(finalref(window), finaltarget(window), cv::NORM_L1) * normalization; //normalized SAD = MAD

	error[0] = err;
	
#if ENABLE_DEBUG_DISPLAY
	Image costimg;
	cv::add(finalref, -1. * finaltarget, costimg);
	cv::abs(costimg);

	cv::namedWindow("ref", cv::WINDOW_NORMAL);
	cv::namedWindow("fref", cv::WINDOW_NORMAL);
	cv::namedWindow("target", cv::WINDOW_NORMAL);
	cv::namedWindow("ftarget", cv::WINDOW_NORMAL);
	cv::namedWindow("wmask", cv::WINDOW_NORMAL);
	cv::namedWindow("cost", cv::WINDOW_NORMAL);
	
	cv::resizeWindow("ref", 200u, 200u);
	cv::resizeWindow("fref", 200u, 200u);
	cv::resizeWindow("target", 200u, 200u);
	cv::resizeWindow("ftarget", 200u, 200u);
	cv::resizeWindow("wmask", 200u, 200u);
	cv::resizeWindow("cost", 200u, 200u);
	
	cv::imshow("fref", finalref);
	cv::imshow("ref", fref);
	cv::imshow("target", ftarget);
	cv::imshow("ftarget", finaltarget);
	cv::imshow("wmask", wmask);
	cv::imshow("cost", costimg);
	
	if (not use_all_micro_image())
	{
		cv::namedWindow("wcost", cv::WINDOW_NORMAL);
		cv::resizeWindow("wcost", 200u, 200u);
		cv::imshow("wcost", costimg(window));
	}
	
	
	if constexpr (useBlur)
	{
		if (mi_i.type != mi_j.type) //if not the same type
		{
			cv::namedWindow("lpedi", cv::WINDOW_NORMAL);
			cv::resizeWindow("lpedi", 200u, 200u);
			cv::imshow("lpedi", lpedi);
		}	
		DEBUG_VAR(sigma_sqr_r);
	}
	
	DEBUG_VAR(v);
	DEBUG_VAR(deltaC);
	DEBUG_VAR(disparity);
	DEBUG_VAR(err);
	DEBUG_VAR(normalization);
	PRINT_DEBUG("---------------------------");
	
	wait();
#endif
	
    return true;
}

template struct DisparityCostError_<false>;
template struct DisparityCostError_<true>;


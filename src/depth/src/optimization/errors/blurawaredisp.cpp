#include "blurawaredisp.h"

#include <pleno/io/printer.h>

#include <pleno/processing/improcess.h>

#define ENABLE_DEBUG_DISPLAY 0
#define SPEED_BEFORE_ACCURACY 0

//******************************************************************************
//******************************************************************************
//******************************************************************************
BlurAwareDisparityCostError::BlurAwareDisparityCostError(
	const Image& img_i_, const Image& img_j_, 
	const MicroImage& mi_i_, const MicroImage& mi_j_, 
	const PlenopticCamera& mfpc_
) : img_i{img_i_.clone()}, img_j{img_j_.clone()}, mi_i{mi_i_}, mi_j{mi_j_}, mfpc{mfpc_} 
{ }

BlurAwareDisparityCostError::BlurAwareDisparityCostError(
	const BlurAwareDisparityCostError& o
) : img_i{o.img_i.clone()}, img_j{o.img_j.clone()}, mi_i{o.mi_i}, mi_j{o.mi_j}, mfpc{o.mfpc} 
{ } 

BlurAwareDisparityCostError::BlurAwareDisparityCostError(
	BlurAwareDisparityCostError&& o
) : img_i{std::move(o.img_i)}, img_j{std::move(o.img_j)}, mi_i{o.mi_i}, mi_j{o.mi_j}, mfpc{o.mfpc} 
{ }

//******************************************************************************
//******************************************************************************
//******************************************************************************
bool BlurAwareDisparityCostError::operator()(
    const VirtualDepth& depth,
	ErrorType& error
) const
{    
    constexpr double epsilon = 2.2204e-16;
    constexpr double maxv = 20.;
    constexpr double lens_border = 1.5; //1.5

    error.setZero();
    
	const double v = depth.v;
	const double radius = mfpc.mia().radius() - lens_border;
    
//0) Check hypotheses:
	//0.1) Discard observation?
    const double eta = (mi_j.center - mi_i.center).norm() / (1.95 * radius);
	if(eta > std::fabs(v) and std::fabs(v) > 2.)
	{
		return false;
	}
	//0.2) Is disparity estimation possible? Is object real?
	if(std::fabs(v) < 2. or mfpc.v2obj(v) < mfpc.focal()) 
	{
		error[0] = 255. / v;
		return true;
	}
    
//1) get ref and edi
	const double A = mfpc.mlaperture(); //mm
	const double d = mfpc.d();			//mm
	const double ai = mfpc.obj2mla(mfpc.focal_plane(mi_i.type));  //mm, negative signed distance to mla
	const double aj = mfpc.obj2mla(mfpc.focal_plane(mi_j.type));  //mm, negative signed distance to mla
	
	const double mij = ((A * A * d) / 2.) * ((1. / ai) - (1. / aj)); //mm²
	const double cij = ((A * A * d * d) / 4.) * ((1. / (ai * ai)) - (1. / (aj * aj))); //mm²

	const double rel_blur = mij * (1. / v) + cij; //mm²	
	
	const bool isOrdered = (rel_blur >= 0.); //(i)-view is more defocused the the (j)-view

	Image ref, edi;	
	if (isOrdered) //(i)-view is more defocused the the (j)-view
	{
		ref = img_i; //(i)-view is ref
		edi = img_j; //(j)-view has to be equally-defocused
	}
	else //(j)-view is more defocused the the (i)-view
	{
#if ENABLE_DEBUG_DISPLAY
		PRINT_DEBUG("Switch views: ref ("<< mi_j.type+1<<"), edi ("<< mi_i.type+1<<")");
#endif 
		ref = img_j; //(j)-view is ref
		edi = img_i; //(i)-view has to be equally-defocused
	}

//2) compute mask	
	//float conversion
	Image fmask, fref, fedi;
	edi.convertTo(fmask, CV_32FC1, 1./255.0); 
	ref.convertTo(fref, CV_32FC1, 1./255.0); 
	edi.convertTo(fedi, CV_32FC1, 1./255.0); 
	
	//apply mask
	trim_float_binarize(fmask, radius); //create mask
#if ENABLE_DEBUG_DISPLAY
	trim_float(fref, radius); //only for vizualisation, apply circular mask on ref
#endif
	
	if(mi_i.type != mi_j.type) //if not the same type
	{	
	//3) compute equally-defocused image
		//3.1) compute sigma_r	
		const double rho_r = mfpc.sensor().metric2pxl(std::sqrt(std::abs(rel_blur)));
		const double kappa = mfpc.params().kappa; 
		const double sigma_r = kappa * rho_r;
		
		#if ENABLE_DEBUG_DISPLAY
			PRINT_DEBUG("r("<< mi_i.type+1<<", "<< mi_j.type+1<<") = " << rel_blur / (mfpc.sensor().scale() * mfpc.sensor().scale()));
			DEBUG_VAR(rho_r);
			DEBUG_VAR(sigma_r);
			
			auto t_start = std::chrono::high_resolution_clock::now();
		#endif
		
		//3.2) blur images, fedi is the final blurred image
		#if SPEED_BEFORE_ACCURACY //blur w/o considering neighbors impact
			cv::GaussianBlur(fedi, fedi, cv::Size{0,0}, sigma_r, sigma_r);
		#else 
			Image bmask, bedi;
			trim_float(fedi, radius); //apply circular mask on edi
			
			cv::GaussianBlur(fmask, bmask, cv::Size{0,0}, sigma_r, sigma_r); //blur mask
			cv::GaussianBlur(fedi, bedi, cv::Size{0,0}, sigma_r, sigma_r); //blur masked image	
			cv::divide(bedi, bmask, fedi); //divide the blurred masked image by the blurred mask 
		#endif
		
		#if ENABLE_DEBUG_DISPLAY
			auto t_end = std::chrono::high_resolution_clock::now();
			double blur_time = std::chrono::duration<double>(t_end-t_start).count();	
			DEBUG_VAR(blur_time);		
			
			trim_float(fedi, radius); //only for vizualisation
		#endif
	}
#if ENABLE_DEBUG_DISPLAY
	else PRINT_DEBUG("Same type ("<< mi_i.type+1<<"), no equally defocused representation");
#endif  	
	
//4) warp image according to depth hypothesis
	//4.1) compute disparity and transformation
	const P2D deltaC = mi_j.center - mi_i.center;
	P2D disparity = deltaC / v; 

	if(not isOrdered) //reoriente the disparity vector
	{
		disparity *= -1.;
	}
		
	cv::Mat M = (cv::Mat_<double>(2,3) << 1., 0., disparity[0], 0., 1., disparity[1]); //Affine transformation
	
	//4.2) warp mask and edi	
	Image wmask, wedi;
	cv::warpAffine(fmask, wmask, M, fmask.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	cv::warpAffine(fedi, wedi, M, fedi.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	
	//re-binarize mask
	Image warpmask;
	trim_float(wmask, radius); //Final mask =  micro-image mask INTER warped masked
	wmask.convertTo(warpmask, CV_8U, 255);
	cv::threshold(warpmask, warpmask, 200, 255, cv::THRESH_BINARY);	

//5) compute cost
	const double normalization = 1. / (cv::sum(wmask)[0] + epsilon); //number of pixel to take into account
	const double cost = cv::norm(fref, wedi, cv::NORM_L1, warpmask) * normalization; //normalized SAD

	error[0] = cost; //* 100.;
	
#if ENABLE_DEBUG_DISPLAY
	Image costimg;
	cv::add(fref, -1. * wedi, costimg, warpmask);
	cv::abs(costimg);

	cv::namedWindow("fref", cv::WINDOW_NORMAL);
	cv::namedWindow("edi", cv::WINDOW_NORMAL);
	cv::namedWindow("fedi", cv::WINDOW_NORMAL);
	cv::namedWindow("warpmask", cv::WINDOW_NORMAL);
	cv::namedWindow("wmask", cv::WINDOW_NORMAL);
	cv::namedWindow("wedi", cv::WINDOW_NORMAL);
	cv::namedWindow("cost", cv::WINDOW_NORMAL);
	
	cv::resizeWindow("fref", 200u, 200u);
	cv::resizeWindow("edi", 200u, 200u);
	cv::resizeWindow("fedi", 200u, 200u);
	cv::resizeWindow("wmask", 200u, 200u);
	cv::resizeWindow("warpmask", 200u, 200u);
	cv::resizeWindow("wedi", 200u, 200u);
	cv::resizeWindow("cost", 200u, 200u);
	
	cv::imshow("fref", fref);
	cv::imshow("edi", edi);
	cv::imshow("fedi", fedi);
	cv::imshow("warpmask", warpmask);
	cv::imshow("wmask", warpmask);
	cv::imshow("wedi", wedi);
	cv::imshow("cost", costimg);

	DEBUG_VAR(v);
	DEBUG_VAR(deltaC);
	DEBUG_VAR(disparity);
	DEBUG_VAR(cost);
	PRINT_DEBUG("---------------------------");
	std::getchar();
#endif
	
    return true;
}

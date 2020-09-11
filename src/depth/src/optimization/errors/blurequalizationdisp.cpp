#include "blurequalizationdisp.h"

#include <pleno/io/printer.h>

#include <pleno/processing/improcess.h>

#define ENABLE_DEBUG_DISPLAY 0

//******************************************************************************
//******************************************************************************
//******************************************************************************
BlurEqualizationDisparityCostError::BlurEqualizationDisparityCostError(
	const Image& img_i_, const Image& img_j_, 
	const MicroImage& mi_i_, const MicroImage& mi_j_, 
	const PlenopticCamera& mfpc_
) : img_i{img_i_.clone()}, img_j{img_j_.clone()}, mi_i{mi_i_}, mi_j{mi_j_}, mfpc{mfpc_} 
{ }

BlurEqualizationDisparityCostError::BlurEqualizationDisparityCostError(
	const BlurEqualizationDisparityCostError& o
) : img_i{o.img_i.clone()}, img_j{o.img_j.clone()}, mi_i{o.mi_i}, mi_j{o.mi_j}, mfpc{o.mfpc} 
{ } 

BlurEqualizationDisparityCostError::BlurEqualizationDisparityCostError(
	BlurEqualizationDisparityCostError&& o
) : img_i{std::move(o.img_i)}, img_j{std::move(o.img_j)}, mi_i{o.mi_i}, mi_j{o.mi_j}, mfpc{o.mfpc} 
{ }

//******************************************************************************
//******************************************************************************
//******************************************************************************
bool BlurEqualizationDisparityCostError::operator()(
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
    const double eta = (mi_j.center - mi_i.center).norm() / (2. * radius);
	if(eta > std::fabs(v) and std::fabs(v) > 2.)
	{
		//PRINT_ERR("Discard observation for hypothesis = " << v << ", eta = " << eta);
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

	const double rho_sqr_r = rel_blur / (mfpc.sensor().scale() * mfpc.sensor().scale()); //pix²
	const double kappa = mfpc.params().kappa; 
	const double sigma_sqr_r = kappa * rho_sqr_r; //pix²
	
	const bool isOrdered = (rel_blur >= 0.); //(i)-view is more defocused the the (j)-view

	Image fref, fedi;	
	if (isOrdered) //(i)-view is more defocused the the (j)-view
	{
		img_i.convertTo(fref, CV_32FC1, 1.f/255.f); //(i)-view is ref
		img_j.convertTo(fedi, CV_32FC1, 1.f/255.f); //(j)-view has to be equally-defocused
	}
	else //(j)-view is more defocused the the (i)-view
	{
#if ENABLE_DEBUG_DISPLAY
		PRINT_DEBUG("Switch views: ref ("<< mi_j.type+1<<"), edi ("<< mi_i.type+1<<")");
#endif 
		img_j.convertTo(fref, CV_32FC1, 1.f/255.f); //(j)-view is ref
		img_i.convertTo(fedi, CV_32FC1, 1.f/255.f); //(i)-view has to be equally-defocused
	}

//2) compute mask	
	Image fmask = Image{fref.size(), CV_32FC1, cv::Scalar::all(1.f)};
	trim_float(fmask, radius);
	
//3) warp image according to depth hypothesis
	//3.1) compute disparity and transformation
	const P2D deltaC = mi_i.center - mi_j.center;
	P2D disparity = deltaC / v; 

	if(not isOrdered) //reoriente the disparity vector
	{
		disparity *= -1.;
	}
		
	cv::Mat M = (cv::Mat_<double>(2,3) << 1., 0., disparity[0], 0., 1., disparity[1]); //Affine transformation
	
	//3.2) warp mask and edi	
	Image wmask, wedi;
	cv::warpAffine(fmask, wmask, M, fmask.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar::all(0.f));
	cv::warpAffine(fedi, wedi, M, fedi.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar::all(0.f));
	
	trim_float(wmask, radius); //Final mask =  micro-image mask INTER warped masked
	
	//3.3) compute laplacian
	Image lpedi; 
	if(mi_i.type != mi_j.type) //if not the same type
	{
		cv::Laplacian(wedi, lpedi, CV_32FC1);
		cv::add(wedi, (std::fabs(sigma_sqr_r) / 4.) * lpedi, wedi);
	}
	
	//3.4) apply mask
	Image finalref = fref.mul(wmask);
	Image finaledi = wedi.mul(wmask); 		

//4) compute cost
	const double summask = cv::sum(wmask)[0];
	const double normalization = 1. / (summask + epsilon); //number of pixel to take into account
	const double cost = cv::norm(finalref, finaledi, cv::NORM_L1) * normalization; //normalized SAD

	error[0] = cost;
	
#if ENABLE_DEBUG_DISPLAY
	Image costimg;
	cv::add(finalref, -1. * finaledi, costimg);
	cv::abs(costimg);

	cv::namedWindow("fref", cv::WINDOW_NORMAL);
	cv::namedWindow("fedi", cv::WINDOW_NORMAL);
	cv::namedWindow("wmask", cv::WINDOW_NORMAL);
	cv::namedWindow("wedi", cv::WINDOW_NORMAL);
	cv::namedWindow("cost", cv::WINDOW_NORMAL);
	
	cv::resizeWindow("fref", 200u, 200u);
	cv::resizeWindow("fedi", 200u, 200u);
	cv::resizeWindow("wmask", 200u, 200u);
	cv::resizeWindow("wedi", 200u, 200u);
	cv::resizeWindow("cost", 200u, 200u);
	
	cv::imshow("fref", finalref);
	cv::imshow("fedi", fedi);
	cv::imshow("wmask", wmask);
	cv::imshow("wedi", finaledi);
	cv::imshow("cost", costimg);
	
	if(mi_i.type != mi_j.type) //if not the same type
	{
		cv::namedWindow("lpedi", cv::WINDOW_NORMAL);
		cv::resizeWindow("lpedi", 200u, 200u);
		cv::imshow("lpedi", lpedi);
	}
	
	DEBUG_VAR(v);
	DEBUG_VAR(sigma_sqr_r);
	DEBUG_VAR(deltaC);
	DEBUG_VAR(disparity);
	DEBUG_VAR(cost);
	DEBUG_VAR(normalization);
	PRINT_DEBUG("---------------------------");
	
	std::getchar();
#endif
	
    return true;
}

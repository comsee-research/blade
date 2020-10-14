#include "disparity.h"

#include <pleno/io/printer.h>

#include <pleno/processing/improcess.h>

//******************************************************************************
//******************************************************************************
//******************************************************************************
DisparityCostError::DisparityCostError(
	const Image& img_i_, const Image& img_j_, 
	const MicroImage& mi_i_, const MicroImage& mi_j_, 
	const PlenopticCamera& mfpc_
) : img_i{img_i_.clone()}, img_j{img_j_.clone()}, mi_i{mi_i_}, mi_j{mi_j_}, mfpc{mfpc_} 
{ }

DisparityCostError::DisparityCostError(
	const DisparityCostError& o
) : img_i{o.img_i.clone()}, img_j{o.img_j.clone()}, mi_i{o.mi_i}, mi_j{o.mi_j}, mfpc{o.mfpc} 
{ } 

DisparityCostError::DisparityCostError(
	DisparityCostError&& o
) : img_i{std::move(o.img_i)}, img_j{std::move(o.img_j)}, mi_i{o.mi_i}, mi_j{o.mi_j}, mfpc{o.mfpc} 
{ }

double DisparityCostError::weight(double) const 
{ 
	return 1.;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
bool DisparityCostError::operator()(
    const VirtualDepth& depth,
	ErrorType& error
) const
{    
    constexpr double threshold_reprojected_pixel = 9.;
    constexpr double epsilon = 2.2204e-16;
    constexpr double lens_border = 1.5; //1.5

    error.setZero();
    
	const double v = depth.v;
	const double radius = mfpc.mia().radius() - lens_border;
    
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
	if(std::fabs(v) < 2. or mfpc.v2obj(v) < mfpc.focal()) 
	{
		error[0] = 255. / v;
		return true;
	}
    
//1) compute ref, target and mask	
	//double conversion
	Image fref, ftarget;
	img_i.convertTo(fref, CV_64FC1, 1./255.0); 
	img_j.convertTo(ftarget, CV_64FC1, 1./255.0); 
	
	//compute mask	
	Image fmask = Image{fref.size(), CV_64FC1, cv::Scalar::all(1.)};
	trim_double(fmask, radius);

//2) warp image according to depth hypothesis
	//2.1) compute transformation
	cv::Mat M = (cv::Mat_<double>(2,3) << 1., 0., disparity[0], 0., 1., disparity[1]); //Affine transformation
	
	//2.2) warp mask and edi	
	Image wmask, wtarget;
	cv::warpAffine(fmask, wmask, M, fmask.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT , cv::Scalar::all(0.));
	cv::warpAffine(ftarget, wtarget, M, ftarget.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	
	trim_double(wmask, radius); //Final mask =  micro-image mask INTER warped masked
	
	//2.3) apply mask
	Image finalref 		= fref.mul(wmask);
	Image finaltarget 	= wtarget.mul(wmask); 	
	
//3) compute error
	const double summask = cv::sum(wmask)[0];
	if (summask < threshold_reprojected_pixel) return false;
	
	const double normalization = 1. / (summask + epsilon); //number of pixel to take into account
	const double err = cv::norm(finalref, finaltarget, cv::NORM_L1) * normalization; //normalized SAD

	error[0] = err;
	
    return true;
}

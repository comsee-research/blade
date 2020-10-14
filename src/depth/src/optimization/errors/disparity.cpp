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
    const double eta = (mi_j.center - mi_i.center).norm() / (2. * radius);
	if(false and eta > std::fabs(v) and std::fabs(v) > 2.)
	{
		return false;
	}
	//0.2) Is disparity estimation possible? Is object real?
	if(std::fabs(v) < 2. or mfpc.v2obj(v) < mfpc.focal()) 
	{
		error[0] = 255. / v;
		return true;
	}
    
//1) compute mask	
	//double conversion
	Image fmask, flhs, frhs;	
	img_i.convertTo(fmask, CV_64FC1, 1./255.0); 
	img_i.convertTo(flhs, CV_64FC1, 1./255.0); 
	img_j.convertTo(frhs, CV_64FC1, 1./255.0); 
	
	//apply mask
	trim_double_binarize(fmask, radius); //create mask

//2) warp image according to depth hypothesis
	//2.1) compute disparity and transformation
	const P2D mli = mfpc.mla().nodeInWorld(mi_i.k, mi_i.l).head(2);
	const P2D mlj = mfpc.mla().nodeInWorld(mi_j.k, mi_j.l).head(2);
	
	const P2D deltaC = (mli - mlj) / mfpc.sensor().scale(); //mi_i.center - mi_j.center; //
	P2D disparity = deltaC / v; 

	cv::Mat M = (cv::Mat_<double>(2,3) << 1., 0., disparity[0], 0., 1., disparity[1]); //Affine transformation
	
	//2.2) warp mask and edi	
	Image wmask, wrhs;
	cv::warpAffine(fmask, wmask, M, fmask.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT , cv::Scalar::all(0.));
	cv::warpAffine(frhs, wrhs, M, frhs.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	
	trim_double(wmask, radius); //Final mask =  micro-image mask INTER warped masked
	
	//2.3) apply mask
	Image finallhs = flhs.mul(wmask);
	Image finalrhs = wrhs.mul(wmask); 	
	
//3) compute error
	const double summask = cv::sum(wmask)[0];
	if (summask < threshold_reprojected_pixel) return false;
	const double normalization = 1. / (summask + epsilon); //number of pixel to take into account
	const double err = cv::norm(finallhs, finalrhs, cv::NORM_L1) * normalization; //normalized SAD

	error[0] = err;
	
    return true;
}

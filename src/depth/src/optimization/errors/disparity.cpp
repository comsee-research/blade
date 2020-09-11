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

//******************************************************************************
//******************************************************************************
//******************************************************************************
bool DisparityCostError::operator()(
    const VirtualDepth& depth,
	ErrorType& error
) const
{    
    constexpr double epsilon = 2.2204e-16;
    constexpr double maxv = 20.;
    constexpr double lens_border = 1.5; //1.5

    error.setZero();
    
	const double v = depth.v;
	const double radius = (mfpc.mia().edge_length()[0] + mfpc.mia().edge_length()[1]) / 4. - lens_border;
    
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
    
//1) compute mask	
	//float conversion
	Image fmask, flhs, frhs;	
	img_i.convertTo(fmask, CV_32FC1, 1./255.0); 
	img_i.convertTo(flhs, CV_32FC1, 1./255.0); 
	img_j.convertTo(frhs, CV_32FC1, 1./255.0); 
	
	//apply mask
	trim_float_binarize(fmask, radius); //create mask

//4) warp image according to depth hypothesis
	//4.1) compute disparity and transformation
	const P2D deltaC = mi_j.center - mi_i.center;
	P2D disparity = deltaC / v; 
	
	//check hypothesis
	//const double ratiodisp = disparity.norm() / (1.95 * radius);
	//if(ratiodisp > 1.) //Is edi reprojected in ref?
	//{
	//	error[0] = std::exp(ratiodisp / 10.); 
	//	return true;
	//}

	cv::Mat M = (cv::Mat_<double>(2,3) << 1., 0., disparity[0], 0., 1., disparity[1]); //Affine transformation
	
	//4.2) warp mask and edi	
	Image wmask, wrhs;
	cv::warpAffine(fmask, wmask, M, fmask.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	cv::warpAffine(frhs, wrhs, M, frhs.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	
	//re-binarize mask
	Image warpmask;
	trim_float(wmask, radius); //Final mask =  micro-image mask INTER warped masked
	wmask.convertTo(warpmask, CV_8U, 255);
	cv::threshold(warpmask, warpmask, 200, 255, cv::THRESH_BINARY);	

//5) compute cost
	const double normalization = 1. / (cv::sum(wmask)[0] + epsilon); //number of pixel to take into account
	const double cost = cv::norm(flhs, wrhs, cv::NORM_L1, warpmask) * normalization; //normalized SAD

	error[0] = cost;
	
    return true;
}

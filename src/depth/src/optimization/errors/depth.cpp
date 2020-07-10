#include "depth.h"

#include <pleno/io/printer.h>

#include <pleno/processing/improcess.h>

#define ENABLE_DEBUG_DISPLAY 0

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
void trim_binarize(Image& img, double radius)
{
	const int width 	= img.cols;
	const int height 	= img.rows;
	const float centerx = width/2.;
	const float centery = height/2.;
	
	uchar * pixel;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		pixel = img.ptr<uchar>(y);
		for(int x = 0 ; x < width ; ++x) //for each column
		{
			if(std::hypot(x-centerx, y-centery) < radius) //in
			{
				pixel[x] = 255; 
			}
			else //out
			{	
				pixel[x] = 0;	
			}			
		}
	}
}

void trim_float(Image& img, double radius)
{
	//TODO: add 0.5 when even
	const int width 	= img.cols;
	const int height 	= img.rows;
	const float centerx = width/2.;
	const float centery = height/2.;
	
	float * pixel;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		pixel = img.ptr<float>(y);
		for(int x = 0 ; x < width ; ++x) //for each column
		{
			if(std::hypot(x-centerx, y-centery) > radius) //out
			{
				pixel[x] = 0.;	
			}			
		}
	}
}


bool BlurAwareDisparityCostError::operator()(
    const Depth& depth,
	ErrorType& error
) const
{    
    error.setZero();
    constexpr double lens_border = 1.;
	const double radius = (mfpc.mia().edge_length()[0] + mfpc.mia().edge_length()[1]) / 4. - lens_border;

//1) get ref and edi
	const double A = mfpc.mlaperture();
	const double d = mfpc.d();
	const double ai = mfpc.obj2mla(mfpc.focal_plane(mi_i.type));
	const double aj = mfpc.obj2mla(mfpc.focal_plane(mi_j.type));
	const double mij = ((A * A * d) / 2.) * ((1. / ai) - (1. / aj));
	const double cij = ((A * A * d * d) / 4.) * ((1. / (ai * ai)) - (1. / (aj * aj)));
	const double v = depth.z; //mfpc.obj2v(depth.z); //DEBUG_ASSERT((v>0), "Virtual depth not positif (problem of conversion): v = " << v);
		
	const double rel_blur = mij * (1. / v) + cij;
	
	Image ref, edi;
	if (rel_blur >= 0.) //(i)-view is more defocused the the (j)-view
	{
		ref = img_i;
		edi = img_j;
	}
	else //(j)-view is more defocused the the (i)-view
	{
		ref = img_j;
		edi = img_i;
	}

//2) compute mask
	Image mask = edi.clone();	

	trim_binarize(mask, radius);	
	trim(edi, radius);	
	trim(ref, radius);
	
	Image fmask, fedi, fref;	
	
	mask.convertTo(fmask, CV_32FC1, 1./255.0);
	ref.convertTo(fref, CV_32FC1, 1./255.0); 
	edi.convertTo(fedi, CV_32FC1, 1./255.0);

	Image bmask, bedi;
	if(mi_i.type != mi_j.type) //if not the same type
	{
	//3) compute equally-defocused image
		//3.1) compute sigma_r	
		const double rho_r = mfpc.sensor().metric2pxl(std::sqrt(std::abs(rel_blur)));
		const double kappa = mfpc.params().kappa;
		const double sigma_r = kappa * rho_r;
		#if ENABLE_DEBUG_DISPLAY
			DEBUG_VAR(sigma_r);
		#endif
		//3.2) blur images
		cv::GaussianBlur(fmask, bmask, cv::Size{0,0}, sigma_r, sigma_r);
		cv::GaussianBlur(fedi, bedi, cv::Size{0,0}, sigma_r, sigma_r);	
		cv::divide(bedi, bmask, fedi); //fedi is the final blurred image
		trim_float(fedi, radius);
	}
	
//4) warp image according to depth hypothesis
	//4.1) compute disparity and transformation
	const P2D deltaC = mi_j.center - mi_i.center;
	P2D disparity = deltaC / v; 
	
	if(rel_blur < 0.) //reoriente the disparity vector
	{
		disparity *= -1.;
	}
	
	//check if reprojection is possible
	//if(disparity.norm() > 2. * radius) { error[0] = 1e5; return true; }
	
	cv::Mat M = (cv::Mat_<double>(2,3) << 1., 0., disparity[0], 0., 1., disparity[1]);
	
	//4.2) warp mask and edi	
	Image wmask, wedi;
	cv::warpAffine(fmask, wmask, M, fmask.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	cv::warpAffine(fedi, wedi, M, fedi.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	
	Image warpmask;
	wmask.convertTo(warpmask, CV_8U, 255);
	cv::threshold(warpmask, warpmask, 252, 255, cv::THRESH_BINARY);	
	trim(warpmask, radius);	

//5) compute cost
	const int nbpixel = cv::countNonZero(warpmask);
	if(nbpixel==0) //no-reprojection
	{
		error[0] = 1e6 / v;
		return true;
	}
	
	const double cost = 255. * cv::norm(fref, wedi, cv::NORM_L1, warpmask) / (static_cast<double>(nbpixel) + 1e-5);	
	error[0] = cost;

#if ENABLE_DEBUG_DISPLAY
	Image costimg;
	cv::add(fref, -1. * wedi, costimg, warpmask);
	cv::abs(costimg);
	
	cv::namedWindow("fref", cv::WINDOW_NORMAL);
	cv::namedWindow("edi", cv::WINDOW_NORMAL);
	cv::namedWindow("fedi", cv::WINDOW_NORMAL);
	cv::namedWindow("warpmask", cv::WINDOW_NORMAL);
	cv::namedWindow("wedi", cv::WINDOW_NORMAL);
	cv::namedWindow("cost", cv::WINDOW_NORMAL);
	
	cv::resizeWindow("fref", 200u, 200u);
	cv::resizeWindow("edi", 200u, 200u);
	cv::resizeWindow("fedi", 200u, 200u);
	cv::resizeWindow("warpmask", 200u, 200u);
	cv::resizeWindow("wedi", 200u, 200u);
	cv::resizeWindow("cost", 200u, 200u);
	
	cv::imshow("fref", fref);
	cv::imshow("edi", edi);
	cv::imshow("fedi", fedi);
	cv::imshow("warpmask", warpmask);
	cv::imshow("wedi", wedi);
	cv::imshow("cost", costimg);

	DEBUG_VAR(v);
	DEBUG_VAR(deltaC);
	DEBUG_VAR(disparity);
	DEBUG_VAR(cost);
	std::getchar();
#endif
	
    return true;
}

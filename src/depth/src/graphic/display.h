#pragma once

#include <pleno/graphic/display.h>

#include "geometry/depth/RawCoarseDepthMap.h"

inline void display(const RawCoarseDepthMap& dm)
{
GUI(
	constexpr std::size_t BORDER_MARGIN = 0;

	const std::size_t kmax = dm.mia().width() - BORDER_MARGIN; 
	const std::size_t kmin = 0 + BORDER_MARGIN;
	const std::size_t lmax = dm.mia().height() - BORDER_MARGIN; 
	const std::size_t lmin = 0 + BORDER_MARGIN;
	
	std::string ss = (dm.is_virtual_depth()?"":"Metric ");
	Image idm = dm.as_image();		
	cv::cvtColor(idm, idm, CV_BGR2RGB);
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name(ss+"Depth Map"),
		idm
  	);

  	// the legend
	constexpr int H = 50;
	const int W = idm.cols;
	
	cv::Mat legend;
	cv::resize(dm.color_map(), legend, cv::Size(W, H), 0, 0, cv::INTER_AREA);
	cv::cvtColor(legend, legend, CV_BGR2RGB);
	
	ss = (dm.is_virtual_depth()?"":" (mm)");
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer())
			.name("Legend"+ss),
		legend,
		0, -2 * H
  	);
  	
  	const double nbsample = dm.is_virtual_depth()? std::ceil(dm.max_depth() - dm.min_depth()) : 15.;
  	const double stepv = dm.is_virtual_depth()? 1. : (dm.max_depth() - dm.min_depth()) / nbsample;
  	const double steppix = W / nbsample;
  	for(double v = dm.min_depth(), offsetpix = -5.; v <= dm.max_depth(); v += stepv, offsetpix += steppix)
  	{
  		std::ostringstream oss;
  		oss.precision((dm.is_virtual_depth()?2:4));
  		oss << v;
  		
  		Viewer::context().layer(Viewer::layer())
  			.font_size(50)
  			.pen_color(v::white)
  			.add_text(
  				offsetpix, - 2.5 * H,
  				oss.str() 			
  			);	  	
  	}
	Viewer::context().layer(Viewer::layer()++).update();  
	
	ss = (dm.is_virtual_depth()?"":"Metric ");  	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			const auto center = dm.mia().nodeInWorld(k,l);	
			std::ostringstream oss;
	  		oss.precision((dm.is_virtual_depth()?3:4));
	  		oss << dm.depth(k,l)*(dm.is_virtual_depth()?10.:1.);		
			
			Viewer::context().layer(Viewer::layer())
  				.font_size(5)
				.name(ss+"Depth Values")
	  			.pen_color(v::white)
	  			.add_text(
	  				center[0], center[1],
	  				oss.str()	  			
	  			);	
		}
	}
	
	Viewer::context().layer(Viewer::layer()++).update();  
);
}

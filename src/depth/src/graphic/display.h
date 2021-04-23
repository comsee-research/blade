#pragma once

#include <pleno/graphic/display.h>
#include "viewer_3d.h"

#include <pleno/geometry/camera/plenoptic.h>

#include "geometry/depth/RawDepthMap.h"
#include "geometry/depth/PointCloud.h"
#include "geometry/depth/convert.h"

#include "types.h"

inline void display(const RawDepthMap& dm, const PlenopticCamera& pcm)
{
GUI(	
	std::string ss = (dm.is_virtual_depth()?"":"Metric ");
	
	const DepthMapImage dmi = to_image(dm, pcm);
	
	Image idm; cv::cvtColor(dmi.image, idm, CV_BGR2RGB);
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name(ss+"Depth Map"),
		idm
  	);

  	// the legend
	constexpr int H = 50;
	const int W = idm.cols;
	
	cv::Mat legend;
	cv::resize(dmi.colormap, legend, cv::Size(W, H), 0, 0, cv::INTER_AREA);
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
  	for (double v = dm.min_depth(), offsetpix = -5.; v <= dm.max_depth(); v += stepv, offsetpix += steppix)
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
	
	if (dm.is_coarse_map())
	{	
		constexpr std::size_t BORDER_MARGIN = 0;

		const std::size_t kmax = dm.width() - BORDER_MARGIN; 
		const std::size_t kmin = 0 + BORDER_MARGIN;
		const std::size_t lmax = dm.height() - BORDER_MARGIN; 
		const std::size_t lmin = 0 + BORDER_MARGIN;
	
		ss = (dm.is_virtual_depth()?"":"Metric ");  	
		for(std::size_t k = kmin; k < kmax; ++k)
		{
			for(std::size_t l = lmin; l < lmax; ++l)
			{
				const auto center = pcm.mia().nodeInWorld(k,l);	
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
	}
);
}

inline void display(int f /* frame */, const PointCloud& pc)
{	
GUI(
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D)
			.layer(Viewer::layer(Viewer::Mode::m3D))
			.name("PointCloud ("+std::to_string(f)+")"), 
		pc
	);
	Viewer::update(Viewer::Mode::m3D);
);
}

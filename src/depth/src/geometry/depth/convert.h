#pragma once

#include <pleno/types.h> //Image, P3D, P2D, RGBA
#include "../../types.h" //DepthMapImage

#include <pleno/geometry/camera/plenoptic.h> //PlenopticCamera

#include "RawDepthMap.h" //RawDepthMap
#include "PointCloud.h" //RawDepthMap

//******************************************************************************
//******************************************************************************
//******************************************************************************
inline DepthMapImage to_image(const RawDepthMap& dm, const PlenopticCamera& model) 
{		
	static constexpr double minimal_resolvable_abs_depth = 2.;
	
	auto is_disparity_estimation_possible = [&](double d) -> bool {
		if(dm.is_metric_depth()) d = model.obj2v(d); //convert to virtual depth
		return (std::fabs(d) > minimal_resolvable_abs_depth);	
	};
	
	auto compute_colomap = [&]() -> cv::Mat {
		cv::Mat colormap;
		
		cv::Mat lut(1, 256, CV_8UC1);
		uchar *ptr = lut.ptr<uchar>(0);
		for(int i = 0; i < 256; ++i) ptr[i] = i;
		
		cv::applyColorMap(lut, colormap, cv::COLORMAP_JET);
				
		if(dm.is_metric_depth() or dm.min_depth() > minimal_resolvable_abs_depth) 
		{
			colormap.at<cv::Vec3b>(0,0) = cv::Vec3b{0,0,0};
		}
		
		for(int i = 0; i < 256; ++i)
		{
			if(not is_disparity_estimation_possible(i * (dm.max_depth() - dm.min_depth()) / 255. + dm.min_depth()))
				colormap.at<cv::Vec3b>(0,i) = cv::Vec3b{0,0,0};		
		}
		
		return colormap;
	};
	
	auto scale_depth = [&](double d) -> std::uint8_t {
		return std::min(255, std::max(0,
			static_cast<int>(255 * (d - dm.min_depth()) / (dm.max_depth() - dm.min_depth()))
		));
	};	
	
	const cv::Mat colormap = compute_colomap();
	
	const std::size_t img_width = model.sensor().width();
	const std::size_t img_height = model.sensor().height();
	
	const std::uint8_t d0 = scale_depth(DepthInfo::NO_DEPTH);

	Image depthmap = Image(img_height, img_width, CV_8UC3, cv::Scalar(d0,d0,d0));
	for(std::size_t k = 0; k < dm.width(); ++k)
	{
		for(std::size_t l = 0; l < dm.height(); ++l)
		{
			const std::uint8_t d = scale_depth(dm.depth(k,l) * (dm.is_valid_depth(dm.depth(k,l))));	
				
			if (dm.is_coarse_map())
			{
				const auto center = model.mia().nodeInWorld(k, l);
				const double radius = model.mia().radius();			
				
				cv::circle(
					depthmap, cv::Point(center[0], center[1]), radius,
					cv::Scalar(d,d,d), CV_FILLED
				);	
			}
			else if (dm.is_refined_map())
			{
				depthmap.at<cv::Vec3b>(l, k) = cv::Vec3b{d, d, d};	//(row,col) access
			}
		}
	}
	
	Image out;
	cv::LUT(depthmap, colormap, out);
	
	return DepthMapImage{out, colormap};				
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
inline PointCloud to_pointcloud(const RawDepthMap& dm, const PlenopticCamera& model, const Image& image)
{
	DEBUG_ASSERT((image.type() == CV_8UC3 or image.type() == CV_8UC1), "Conversion need image format U8C3/1");
	
	
	const auto& sensor 	= model.sensor();
	const auto& mia 	= model.mia();
	
	const int W 	= static_cast<int>(sensor.width());
	const int H 	= static_cast<int>(sensor.height());
	const double R 	= mia.radius();// + 1;
	
	PointCloud pc{static_cast<std::size_t>(W * H)};

	//for each micro-image
	for (std::size_t k= 0; k < mia.width(); ++k)
	{
		for (std::size_t l = 0; l < mia.height(); ++l)
		{	
			if (dm.is_coarse_map() and dm.depth(k, l) == DepthInfo::NO_DEPTH) continue;
			
			const auto c = mia.nodeInWorld(k,l);
			
			const int umin = std::max(static_cast<int>(c[0] - R), 0);
			const int umax = std::min(static_cast<int>(c[0] + R), W);
			const int vmin = std::max(static_cast<int>(c[1] - R), 0);
			const int vmax = std::min(static_cast<int>(c[1] + R), H);
			
			//for each pixel
			for (int u = umin; u < umax; ++u) //col
			{
				for (int v = vmin; v < vmax; ++v) //row
				{
					//get depth
					const double depth = dm.is_refined_map() ? dm.depth(u,v) : dm.depth(k,l);
					if (depth == DepthInfo::NO_DEPTH) continue;
									
					//get center pixel to raytrace from
					const P2D pixel = {u+0.5, v+0.5};
					if ((pixel - c).norm() > R) continue; //out of distance
					
					const P2D kl = model.mi2ml(k, l); //get ml indexes
					
					//raytrace
					Ray3D ray; //in CAMERA frame
					if (model.raytrace(pixel, kl[0], kl[1], ray))
					{
						//get depth plane
						PlaneCoefficients plane; plane << 0., 0., 1., -depth;
						
						//get position
						const P3D point = line_plane_intersection(plane, ray);
						
						//get color
						const RGBA color = [&]() -> RGBA {
							if (image.type() == CV_8UC3)
							{
								const auto bgr = image.at<cv::Vec3b>(v,u); //(row,col) access
								return RGBA{
									static_cast<double>(bgr[2]), 
									static_cast<double>(bgr[1]), 
									static_cast<double>(bgr[0]), 
									255.
								}; 
							}
							else if (image.type() == CV_8UC1)
							{
								const double g = static_cast<double>(image.at<uchar>(v,u)); //(row,col) access
								return RGBA{g, g, g, 255.}; 
							}
						}();
						
						//add to pointcloud
						pc.add(point, pixel, color);					
					}
				}
			}
		}
	}
	
	pc.shrink();
	return pc;			
}





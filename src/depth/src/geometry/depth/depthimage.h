#pragma once

#include <utility>

#include <pleno/types.h>
#include <pleno/geometry/camera/plenoptic.h> //PlenopticCamera

#include "geometry/depth/depthmap.h"
#include "geometry/depth/pointcloud.h"

struct DepthMapImage {
private:
	static constexpr double minimal_resolvable_abs_depth 	= 2.;
	static constexpr double auto_dist_from_data 			= -1.;
	
public:
	Image depthmap; //CV_32F, to be save in .exr
	Image image;
	cv::Mat colormap;
		
public:
	DepthMapImage(const Image& dm, const cv::Mat& cm);
	DepthMapImage(const Image& dm, const Image& im, const cv::Mat& cm);
	
	DepthMapImage(
		const DepthMap& dm, const PlenopticCamera& model,
		double mind = auto_dist_from_data, double maxd = auto_dist_from_data
	);
	DepthMapImage(
		const PointCloud& pc, const PlenopticCamera& model,
		double mind = auto_dist_from_data, double maxd = auto_dist_from_data,
		bool use_median_as_interpolation_method = true
	);
	
private:
	void compute_colormap(
		const DepthMap& dm, const PlenopticCamera& model,
		double mind = auto_dist_from_data, double maxd = auto_dist_from_data
	);
	void compute_colormap(
		const PointCloud& pc, const PlenopticCamera& model,
		double mind = auto_dist_from_data, double maxd = auto_dist_from_data
	);
	
	std::uint8_t scale_depth(double mind, double maxd, double d) const;	
	
	bool is_disparity_estimation_possible(const DepthMap& dm, const PlenopticCamera& model, double d) const;
	bool is_disparity_estimation_possible(const PointCloud& pc, const PlenopticCamera& model, double d) const;
};

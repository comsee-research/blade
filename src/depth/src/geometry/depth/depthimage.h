#pragma once


#include <pleno/types.h>
#include <pleno/geometry/camera/plenoptic.h> //PlenopticCamera

#include "geometry/depth/depthmap.h"
#include "geometry/depth/pointcloud.h"

struct DepthMapImage {
private:
	static constexpr double minimal_resolvable_abs_depth = 2.;
	
public:
	Image image;
	cv::Mat colormap;
	
public:
	DepthMapImage(const Image& im, const cv::Mat& cm);
	DepthMapImage(const DepthMap& dm, const PlenopticCamera& model);
	DepthMapImage(const PointCloud& pc, const PlenopticCamera& model);
	
private:
	void compute_colormap(const DepthMap& dm, const PlenopticCamera& model);
	void compute_colormap(const PointCloud& pc, const PlenopticCamera& model);
	
	std::uint8_t scale_depth(double mind, double maxd, double d) const;	
	
	bool is_disparity_estimation_possible(const DepthMap& dm, const PlenopticCamera& model, double d) const;
	bool is_disparity_estimation_possible(const PointCloud& pc, const PlenopticCamera& model, double d) const;
};

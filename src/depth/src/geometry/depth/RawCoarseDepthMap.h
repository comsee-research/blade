#pragma once

#include <opencv2/imgproc.hpp>

#include <pleno/types.h>

#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/mia.h>

struct DepthInfo {
	static constexpr double NO_DEPTH = 0.;
	
	enum State : std::uint8_t {
		UNINITIALIZED = 0, INITIALIZED, COMPUTED
	};	
	
	double depth = NO_DEPTH;
	State state = UNINITIALIZED;	
};

class RawCoarseDepthMap {	
	using DepthMapContainer = Eigen::Matrix<DepthInfo, Eigen::Dynamic, Eigen::Dynamic>;
	
	static constexpr double minimal_resolvable_abs_depth = 2.;

	const PlenopticCamera& mfpc;	
	const std::size_t width;
	const std::size_t height;
	
	DepthMapContainer map;
	
	double min_depth_;
	double max_depth_;
	
	bool use_virtual_depth;

	cv::Mat colormap;
		
public:
//******************************************************************************
	RawCoarseDepthMap(const PlenopticCamera& pcm, double mind = 2., double maxd = 15., bool useVirtualDepth = true);
	RawCoarseDepthMap(const RawCoarseDepthMap& o);
	RawCoarseDepthMap(RawCoarseDepthMap&& o);

//******************************************************************************		
	const MIA& mia() const;
	const cv::Mat& color_map() const;

	double depth(std::size_t k, std::size_t l) const;
	double& depth(std::size_t k, std::size_t l);
	
	DepthInfo::State state(std::size_t k, std::size_t l) const;
	DepthInfo::State& state(std::size_t k, std::size_t l);
	
	DepthInfo depth_with_info(std::size_t k, std::size_t l) const;
	DepthInfo& depth_with_info(std::size_t k, std::size_t l);
	
	double min_depth() const;
	void min_depth(double mind);
	double max_depth() const;
	void max_depth(double maxd);
	
//******************************************************************************	
	bool is_virtual_depth() const;
	bool is_metric_depth() const;
	bool is_valid_depth(double d) const;
	
//******************************************************************************	
	Image as_image() const;
	RawCoarseDepthMap as_metric() const;
	
protected:
//******************************************************************************
	void compute_colomap();
	
	std::uint8_t scale_depth(double d) const;
	
	bool is_depth_out_of_bounds(double d) const;
	bool is_disparity_estimation_possible(double d) const;
			
};

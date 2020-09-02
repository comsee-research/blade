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

	const PlenopticCamera& mfpc;	
	const std::size_t width;
	const std::size_t height;
	
	DepthMapContainer map;
	
	double min_depth_;
	double max_depth_;
	
	bool use_virtual_depth;

	cv::Mat colormap;
		
public:
	RawCoarseDepthMap(const PlenopticCamera& pcm, double mind = 2., double maxd = 15., bool useVirtualDepth = true) 
	: 	mfpc{pcm}, 
		width{pcm.sensor().width()}, height{pcm.sensor().height()}, 
		map{pcm.mia().width(), pcm.mia().height()},
		min_depth_{mind}, max_depth_{maxd},
		use_virtual_depth{useVirtualDepth}
	{
		compute_colomap();	
	}
	
	RawCoarseDepthMap(const RawCoarseDepthMap& o)
	:	mfpc{o.mfpc}, width{o.width}, height{o.height},
		map{o.mia().width(), o.mia().height()},
		min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
		use_virtual_depth{o.use_virtual_depth}
	{
		compute_colomap();
			
		for(std::size_t k = 0; k < mia().width(); ++k)
		{
			for(std::size_t l = 0; l < mia().height(); ++l)
			{
				depth(k,l) = o.depth(k,l);
				state(k,l) = o.state(k,l);
			}
		}
	}
	
	RawCoarseDepthMap(RawCoarseDepthMap&& o)
	:	mfpc{o.mfpc}, width{o.width}, height{o.height},
		map{std::move(o.map)},
		min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
		use_virtual_depth{o.use_virtual_depth},
		colormap{std::move(o.colormap)}
	{
	
	}
	
		
	const MIA& mia() const { return mfpc.mia(); }
	const cv::Mat& color_map() const { return colormap; }

	double depth(std::size_t k, std::size_t l) const { return map(k,l).depth; }
	double& depth(std::size_t k, std::size_t l) { return map(k,l).depth; }
	
	DepthInfo::State state(std::size_t k, std::size_t l) const { return map(k,l).state; }
	DepthInfo::State& state(std::size_t k, std::size_t l) { return map(k,l).state; }
	
	DepthInfo depth_with_info(std::size_t k, std::size_t l) const { return map(k,l); }
	DepthInfo& depth_with_info(std::size_t k, std::size_t l) { return map(k,l); }
	
	double min_depth() const { return min_depth_; }
	void min_depth(double mind) { min_depth_ = mind; }
	double max_depth() const { return max_depth_; }
	void max_depth(double maxd) { max_depth_ = maxd; }
	
	bool is_virtual_depth() const { return use_virtual_depth; }
	bool is_valid_depth(double d) const { return (is_disparity_estimation_possible(d) and not(is_depth_out_of_bounds(d))); }
	
	Image as_image() const {		
		const std::uint8_t d0 = scale_depth(DepthInfo::NO_DEPTH);
		
		Image depthmap = Image(height, width, CV_8UC3, cv::Scalar(d0,d0,d0));
		for(std::size_t k = 0; k < mia().width(); ++k)
		{
			for(std::size_t l = 0; l < mia().height(); ++l)
			{
				const auto center = mia().nodeInWorld(k,l);
				const double radius = mia().radius();
				const std::uint8_t d = scale_depth(depth(k,l) * (is_valid_depth(depth(k,l))));				
				
				cv::circle(
					depthmap, cv::Point(center[0], center[1]), radius,
					cv::Scalar(d,d,d), CV_FILLED
				);	
			}
		}
		
		Image out;
		cv::LUT(depthmap, colormap, out);
		
		return out;				
	}
	
	RawCoarseDepthMap as_metric() const {
		if(not is_virtual_depth()) return RawCoarseDepthMap(*this); //already metric map
		
		//else, convert to metric
		const double mind = mfpc.v2obj(max_depth());
		const double maxd = std::min(mfpc.v2obj(min_depth()), 5e3); //max 5m
		RawCoarseDepthMap mdm{mfpc, mind, maxd, false};

		for(std::size_t k = 0; k < mia().width(); ++k)
		{
			for(std::size_t l = 0; l < mia().height(); ++l)
			{
				if(depth(k,l) != DepthInfo::NO_DEPTH)
				{
					mdm.depth(k,l) = mfpc.v2obj(depth(k,l));
				}
				mdm.state(k,l) = state(k,l);
			}
		}
		
		return mdm;	
	}
	
protected:
	void compute_colomap()
	{
		cv::Mat lut(1, 256, CV_8UC1);
		uchar *ptr = lut.ptr<uchar>(0);
		for(int i = 0; i < 256; ++i) ptr[i] = i;
		
		cv::applyColorMap(lut, colormap, cv::COLORMAP_JET);
				
		if(not is_virtual_depth() or min_depth() > 2.) colormap.at<cv::Vec3b>(0,0) = cv::Vec3b{0,0,0};
		
		for(int i = 0; i < 256; ++i)
		{
			//if(std::fabs(i * (max_depth() - min_depth()) / 255. + min_depth()) <= 2.)
			if(not is_disparity_estimation_possible(i * (max_depth() - min_depth()) / 255. + min_depth()))
				colormap.at<cv::Vec3b>(0,i) = cv::Vec3b{0,0,0};		
		}
	}
	
	std::uint8_t scale_depth(double d) const 
	{
		return std::min(255, std::max(0,
			static_cast<int>(255 * (d - min_depth()) / (max_depth() - min_depth())) 			
		));
	}
	
	bool is_depth_out_of_bounds(double d) const 
	{
		return (d >= max_depth() or d < min_depth());
	}
	
	bool is_disparity_estimation_possible(double d) const 
	{
		if(not is_virtual_depth()) d = mfpc.obj2v(d); //convert to virtual depth
		return (std::fabs(d) > 2.0);	
	}
			
};

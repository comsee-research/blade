#pragma once

#include <opencv2/imgproc.hpp>

#include <pleno/types.h>

#include <pleno/io/archive.h>

#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/mia.h>

struct DepthInfo {
	static constexpr double NO_DEPTH = 0.;
	static constexpr double NO_CONFIDENCE = -1.;
	
	enum State : std::uint16_t {
		UNINITIALIZED = 0, INITIALIZED, COMPUTED
	};	
	
	double depth = NO_DEPTH;
	double confidence = NO_CONFIDENCE;
	State state = UNINITIALIZED;	
};

void save(v::OutputArchive& archive, const DepthInfo& di);
void load(v::InputArchive& archive, DepthInfo& di);

//******************************************************************************
//******************************************************************************
//******************************************************************************
class RawDepthMap {
public:	
	using DepthMapContainer = Eigen::Matrix<DepthInfo, Eigen::Dynamic /* row */, Eigen::Dynamic /* col */>;
	
	enum DepthType : bool { VIRTUAL = true, METRIC = false };
	enum MapType : bool { COARSE = true, REFINED = false };
private:
	//TODO: remove dependancies to mfpc + graphic
	static constexpr double minimal_resolvable_abs_depth = 2.;
	const PlenopticCamera& mfpc; //for v2obj, obj2v conversion	
		
	std::size_t img_width;
	std::size_t img_height;
	
	DepthMapContainer map;
	
	double min_depth_;
	double max_depth_;
	
	DepthType depth_type;
	MapType map_type;

	cv::Mat colormap;
		
public:
//******************************************************************************
	//TODO: remove dependancies to mfpc
	RawDepthMap(const PlenopticCamera& pcm, double mind = 2., double maxd = 15., DepthType dtype = VIRTUAL, MapType mtype = COARSE);
	RawDepthMap(const RawDepthMap& o);
	RawDepthMap(RawDepthMap&& o);

//******************************************************************************	
	const PlenopticCamera& pcm() const;	
		
	const cv::Mat& color_map() const;
	
	std::size_t width() const;
	std::size_t height() const;

	double depth(std::size_t k, std::size_t l) const;
	double& depth(std::size_t k, std::size_t l);
	
	double confidence(std::size_t k, std::size_t l) const;
	double& confidence(std::size_t k, std::size_t l);
	
	DepthInfo::State state(std::size_t k, std::size_t l) const;
	DepthInfo::State& state(std::size_t k, std::size_t l);
	
	DepthInfo depth_with_info(std::size_t k, std::size_t l) const;
	DepthInfo& depth_with_info(std::size_t k, std::size_t l);
	
	double min_depth() const;
	void min_depth(double mind);
	
	double max_depth() const;
	void max_depth(double maxd);
	
	void copy_from(const RawDepthMap& o);
	void copy_to(RawDepthMap& o) const;
	
//******************************************************************************	
	bool is_virtual_depth() const;
	bool is_metric_depth() const;
	
	bool is_valid_depth(double d) const;
	
//******************************************************************************
	bool is_coarse_map() const;
	bool is_refined_map() const;
	
//******************************************************************************	
	Image to_image() const;
	
	RawDepthMap to_metric(const PlenopticCamera& pcm) const;
	RawDepthMap to_virtual(const PlenopticCamera& pcm) const;

protected:
//******************************************************************************
	void compute_colomap();
	
	std::uint8_t scale_depth(double d) const;
	
	bool is_depth_out_of_bounds(double d) const;
	bool is_disparity_estimation_possible(double d) const;	
	
//******************************************************************************
	friend void save(v::OutputArchive& archive, const RawDepthMap& dm);
	friend void load(v::InputArchive& archive, RawDepthMap& dm);		
};

void save(v::OutputArchive& archive, const RawDepthMap& dm);
void load(v::InputArchive& archive, RawDepthMap& dm);

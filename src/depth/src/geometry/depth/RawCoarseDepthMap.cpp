#include "RawCoarseDepthMap.h"

//******************************************************************************
// constructors
//******************************************************************************
RawCoarseDepthMap::RawCoarseDepthMap(const PlenopticCamera& pcm, double mind, double maxd, bool useVirtualDepth) 
: 	mfpc{pcm}, 
	width{pcm.sensor().width()}, height{pcm.sensor().height()}, 
	map{pcm.mia().width(), pcm.mia().height()},
	min_depth_{mind}, max_depth_{maxd},
	use_virtual_depth{useVirtualDepth}
{
	compute_colomap();	
}

RawCoarseDepthMap::RawCoarseDepthMap(const RawCoarseDepthMap& o)
:	mfpc{o.mfpc}, width{o.width}, height{o.height},
	map{o.mia().width(), o.mia().height()},
	min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
	use_virtual_depth{o.use_virtual_depth}
{
	compute_colomap();
	
	//copy depth map data
	copy_map(o);
}

RawCoarseDepthMap::RawCoarseDepthMap(RawCoarseDepthMap&& o)
:	mfpc{o.mfpc}, width{o.width}, height{o.height},
	map{std::move(o.map)},
	min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
	use_virtual_depth{o.use_virtual_depth},
	colormap{std::move(o.colormap)}
{

}

//******************************************************************************
// accessors
//******************************************************************************
const PlenopticCamera& RawCoarseDepthMap::pcm() const { return mfpc; }	
const MIA& RawCoarseDepthMap::mia() const { return mfpc.mia(); }
const cv::Mat& RawCoarseDepthMap::color_map() const { return colormap; }

void RawCoarseDepthMap::copy_map(const RawCoarseDepthMap& o)
{
	for(std::size_t k = 0; k < mia().width(); ++k)
	{
		for(std::size_t l = 0; l < mia().height(); ++l)
		{
			depth(k,l) = o.depth(k,l);
			state(k,l) = o.state(k,l);
		}
	}
}

double RawCoarseDepthMap::depth(std::size_t k, std::size_t l) const { return map(k,l).depth; }
double& RawCoarseDepthMap::depth(std::size_t k, std::size_t l) { return map(k,l).depth; }

DepthInfo::State RawCoarseDepthMap::state(std::size_t k, std::size_t l) const { return map(k,l).state; }
DepthInfo::State& RawCoarseDepthMap::state(std::size_t k, std::size_t l) { return map(k,l).state; }

DepthInfo RawCoarseDepthMap::depth_with_info(std::size_t k, std::size_t l) const { return map(k,l); }
DepthInfo& RawCoarseDepthMap::depth_with_info(std::size_t k, std::size_t l) { return map(k,l); }

double RawCoarseDepthMap::min_depth() const { return min_depth_; }
void RawCoarseDepthMap::min_depth(double mind) { min_depth_ = mind; compute_colomap();}
double RawCoarseDepthMap::max_depth() const { return max_depth_; }
void RawCoarseDepthMap::max_depth(double maxd) { max_depth_ = maxd; compute_colomap();}


//******************************************************************************
// export functions
//******************************************************************************
Image RawCoarseDepthMap::as_image() const 
{		
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

RawCoarseDepthMap RawCoarseDepthMap::as_metric() const 
{
	if(is_metric_depth()) return RawCoarseDepthMap(*this); //already metric map
	
	//else, convert to metric
	const double mind = mfpc.v2obj(max_depth());
	const double maxd = std::min(mfpc.v2obj(min_depth()), 100. * mfpc.focal()); //max 100 * F
	RawCoarseDepthMap mdm{mfpc, mind, maxd, false};

	//copy and convert depth map data
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


//******************************************************************************
// Helper functions
//******************************************************************************
bool RawCoarseDepthMap::is_virtual_depth() const { return use_virtual_depth; }
bool RawCoarseDepthMap::is_metric_depth() const { return not is_virtual_depth(); }
bool RawCoarseDepthMap::is_valid_depth(double d) const { return (is_disparity_estimation_possible(d) and not(is_depth_out_of_bounds(d))); }

void RawCoarseDepthMap::compute_colomap()
{
	cv::Mat lut(1, 256, CV_8UC1);
	uchar *ptr = lut.ptr<uchar>(0);
	for(int i = 0; i < 256; ++i) ptr[i] = i;
	
	cv::applyColorMap(lut, colormap, cv::COLORMAP_JET);
			
	if(is_metric_depth() or min_depth() > minimal_resolvable_abs_depth) colormap.at<cv::Vec3b>(0,0) = cv::Vec3b{0,0,0};
	
	for(int i = 0; i < 256; ++i)
	{
		if(not is_disparity_estimation_possible(i * (max_depth() - min_depth()) / 255. + min_depth()))
			colormap.at<cv::Vec3b>(0,i) = cv::Vec3b{0,0,0};		
	}
}

std::uint8_t RawCoarseDepthMap::scale_depth(double d) const 
{
	return std::min(255, std::max(0,
		static_cast<int>(255 * (d - min_depth()) / (max_depth() - min_depth())) 			
	));
}

bool RawCoarseDepthMap::is_depth_out_of_bounds(double d) const 
{
	return (d >= max_depth() or d < min_depth());
}

bool RawCoarseDepthMap::is_disparity_estimation_possible(double d) const 
{
	if(is_metric_depth()) d = mfpc.obj2v(d); //convert to virtual depth
	return (std::fabs(d) > minimal_resolvable_abs_depth);	
}


//******************************************************************************
// save/load functions
//******************************************************************************
void save(v::OutputArchive& archive, const DepthInfo& di)
{
	std::uint16_t istate = di.state;
	archive
		("depth", di.depth)
		("state", istate);
}
void load(v::InputArchive& archive, DepthInfo& di)
{
	std::uint16_t istate;
	archive
		("depth", di.depth)
		("state", istate);
		
	di.state = DepthInfo::State(istate);
}
void save(v::OutputArchive& archive, const RawCoarseDepthMap& dm)
{
	int cols = dm.map.cols();
	int rows =  dm.map.rows();
	int n = cols * rows;
	
	AlignedVector<DepthInfo> depths(n);
	RawCoarseDepthMap::DepthMapContainer::Map(depths.data(),1,n) = dm.map;
	
	archive
		("map", depths)
		("cols", cols)
		("rows", rows);
}
void load(v::InputArchive& archive, RawCoarseDepthMap& dm)
{
	AlignedVector<DepthInfo> depths;
	int cols, rows;
	archive
		("map", depths)
		("cols", cols)
		("rows", rows);
		
	dm.map = Eigen::Map<RawCoarseDepthMap::DepthMapContainer>(&depths[0], rows, cols);
}

#include "RawDepthMap.h"

//******************************************************************************
// constructors
//******************************************************************************
RawDepthMap::RawDepthMap(
	const PlenopticCamera& pcm, double mind, double maxd,
	DepthType dtype, MapType mtype
) : mfpc{pcm}, //TODO: remove dependancies to mfpc
	//image size (TODO: remove)
	img_width{pcm.sensor().width()}, img_height{pcm.sensor().height()},
	//map
	map{
		mtype == COARSE ? 
			pcm.mia().height() : pcm.sensor().height(), /* rows */
		mtype == COARSE ? 	
			pcm.mia().width() : pcm.sensor().width() /* cols */
	},
	//distances
	min_depth_{mind}, max_depth_{maxd},
	//depth map configuration
	depth_type{dtype}, map_type{mtype}
{
	compute_colomap(); //TODO: remove dependancies to graphic element
}

RawDepthMap::RawDepthMap(const RawDepthMap& o)
:	mfpc{o.mfpc}, //TODO: remove dependancies to mfpc
	//image size (TODO: remove)
	img_width{o.img_width}, img_height{o.img_height},
	//map
	map{o.height(), o.width()},
	//distances
	min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
	//depth map configuration
	depth_type{o.depth_type}, map_type{o.map_type}
{
	compute_colomap(); //TODO: remove dependancies to graphic element
	copy_from(o); //copy depth map data
}

RawDepthMap::RawDepthMap(RawDepthMap&& o)
:	mfpc{o.mfpc}, //TODO: remove dependancies to mfpc
	//image size (TODO: remove)
	img_width{o.img_width}, img_height{o.img_height},
	//map
	map{std::move(o.map)},
	//distances
	min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
	//depth map configuration
	depth_type{o.depth_type}, map_type{o.map_type},
	//TODO: remove dependancies to graphic element
	colormap{std::move(o.colormap)}
{
}

//******************************************************************************
// accessors
//******************************************************************************
//TODO: remove dependancies to mfpc
const PlenopticCamera& RawDepthMap::pcm() const { return mfpc; }
//TODO: remove dependancies to graphic element
const cv::Mat& RawDepthMap::color_map() const { return colormap; } 

void RawDepthMap::copy_to(RawDepthMap& o) const { o.copy_from(*this); }
void RawDepthMap::copy_from(const RawDepthMap& o)
{
	DEBUG_ASSERT((o.map_type == map_type), "Can't copy data from different depth map type.");
	
	for(std::size_t k = 0; k < width(); ++k)
	{
		for(std::size_t l = 0; l < height(); ++l)
		{
			depth(k,l) 		= o.depth(k,l);
			confidence(k,l) = o.confidence(k,l);
			state(k,l) 		= o.state(k,l);
		}
	}
}

std::size_t RawDepthMap::width() const { return map.cols(); }
std::size_t RawDepthMap::height() const { return map.rows(); }

double RawDepthMap::depth(std::size_t k, std::size_t l) const { return map(l,k).depth; }
double& RawDepthMap::depth(std::size_t k, std::size_t l) { return map(l,k).depth; }

double RawDepthMap::confidence(std::size_t k, std::size_t l) const { return map(l,k).confidence; }
double& RawDepthMap::confidence(std::size_t k, std::size_t l) { return map(l,k).confidence; }

DepthInfo::State RawDepthMap::state(std::size_t k, std::size_t l) const { return map(l,k).state; }
DepthInfo::State& RawDepthMap::state(std::size_t k, std::size_t l) { return map(l,k).state; }

DepthInfo RawDepthMap::depth_with_info(std::size_t k, std::size_t l) const { return map(l,k); }
DepthInfo& RawDepthMap::depth_with_info(std::size_t k, std::size_t l) { return map(l,k); }

double RawDepthMap::min_depth() const { return min_depth_; }
void RawDepthMap::min_depth(double mind) { min_depth_ = mind; compute_colomap();}
double RawDepthMap::max_depth() const { return max_depth_; }
void RawDepthMap::max_depth(double maxd) { max_depth_ = maxd; compute_colomap();}

//******************************************************************************
// export functions
//******************************************************************************
//TODO: move to a renderer class
//TODO: remove dependancies to graphic element
Image RawDepthMap::to_image() const 
{		
	const std::uint8_t d0 = scale_depth(DepthInfo::NO_DEPTH);

	Image depthmap = Image(img_height, img_width, CV_8UC3, cv::Scalar(d0,d0,d0));
	for(std::size_t k = 0; k < width(); ++k)
	{
		for(std::size_t l = 0; l < height(); ++l)
		{
			const std::uint8_t d = scale_depth(depth(k,l) * (is_valid_depth(depth(k,l))));	
				
			if (is_coarse_map())
			{
				const auto center = pcm().mia().nodeInWorld(k, l);
				const double radius = pcm().mia().radius();			
				
				cv::circle(
					depthmap, cv::Point(center[0], center[1]), radius,
					cv::Scalar(d,d,d), CV_FILLED
				);	
			}
			else if (is_dense_map())
			{
				depthmap.at<cv::Vec3b>(l, k) = d, d, d;	//(row,col) access
			}
		}
	}
	
	Image out;
	cv::LUT(depthmap, colormap, out);
	
	return out;				
}

RawDepthMap RawDepthMap::to_metric(const PlenopticCamera& pcm) const 
{
	if(is_metric_depth()) return RawDepthMap(*this); //already metric map
	
	//else, convert to metric
	const double mind = pcm.v2obj(max_depth());
	const double maxd = std::min(std::fabs(pcm.v2obj(min_depth())), 100. * pcm.focal()); //max 100 * F
	
	RawDepthMap mdm{pcm, mind, maxd, METRIC, map_type};

	//copy and convert depth map data
	for(std::size_t k = 0; k < width(); ++k)
	{
		for(std::size_t l = 0; l < height(); ++l)
		{
			if(depth(k,l) != DepthInfo::NO_DEPTH)
			{
				mdm.depth(k,l) = pcm.v2obj(depth(k,l));
			}
			mdm.state(k,l) = state(k,l);
			mdm.confidence(k,l) = confidence(k,l);
		}
	}
	
	return mdm;	
}

RawDepthMap RawDepthMap::to_virtual(const PlenopticCamera& pcm) const 
{
	if(is_virtual_depth()) return RawDepthMap(*this); //already virtual map
	
	//else, convert to virtual
	const double minv = pcm.obj2v(max_depth());
	const double maxv = std::max(std::fabs(pcm.obj2v(min_depth())), pcm.obj2v(4. * std::ceil(pcm.focal())));
	
	RawDepthMap vdm{pcm, minv, maxv, VIRTUAL, map_type};

	//copy and convert depth map data
	for(std::size_t k = 0; k < width(); ++k)
	{
		for(std::size_t l = 0; l < height(); ++l)
		{
			if(depth(k,l) != DepthInfo::NO_DEPTH)
			{
				vdm.depth(k,l) = pcm.obj2v(depth(k,l));
			}
			vdm.state(k,l) = state(k,l);
			vdm.confidence(k,l) = confidence(k,l);
		}
	}
	
	return vdm;	
}


//******************************************************************************
// Helper functions
//******************************************************************************
bool RawDepthMap::is_virtual_depth() const { return (depth_type == VIRTUAL); }
bool RawDepthMap::is_metric_depth() const { return not is_virtual_depth(); }

bool RawDepthMap::is_coarse_map() const { return (map_type == COARSE); }
bool RawDepthMap::is_dense_map() const { return not is_coarse_map(); }

bool RawDepthMap::is_valid_depth(double d) const { return (is_disparity_estimation_possible(d) and not(is_depth_out_of_bounds(d))); }

//TODO: remove dependancies to graphic element
void RawDepthMap::compute_colomap()
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
//TODO: remove dependancies to graphic element
std::uint8_t RawDepthMap::scale_depth(double d) const 
{
	return std::min(255, std::max(0,
		static_cast<int>(255 * (d - min_depth()) / (max_depth() - min_depth())) 			
	));
}

bool RawDepthMap::is_depth_out_of_bounds(double d) const 
{
	return (d >= max_depth() or d < min_depth());
}

//TODO: move to a renderer class
bool RawDepthMap::is_disparity_estimation_possible(double d) const 
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
		("confidence", di.confidence)
		("state", istate);
}
void load(v::InputArchive& archive, DepthInfo& di)
{
	std::uint16_t istate;
	archive
		("depth", di.depth)
		("confidence", di.confidence)
		("state", istate);
		
	di.state = DepthInfo::State(istate);
}
void save(v::OutputArchive& archive, const RawDepthMap& dm)
{
	int cols = dm.map.cols();
	int rows =  dm.map.rows();
	int n = cols * rows;
	
	AlignedVector<DepthInfo> depths(n);
	RawDepthMap::DepthMapContainer::Map(depths.data(),1,n) = dm.map;
	
	bool metric = dm.is_metric_depth();
	bool coarse = dm.is_coarse_map();
	
	double min = dm.min_depth();
	double max = dm.max_depth();
	
	archive
		("map", depths)
		("cols", cols)
		("rows", rows)
		("min", min)
		("max", max)
		("metric", metric)
		("coarse", coarse);
}
void load(v::InputArchive& archive, RawDepthMap& dm)
{
	AlignedVector<DepthInfo> depths;
	int cols, rows;
	bool metric, coarse;
	double min, max;
	
	archive
		("map", depths)
		("cols", cols)
		("rows", rows)
		("min", min)
		("max", max)
		("metric", metric)
		("coarse", coarse);
		
	dm.map = Eigen::Map<RawDepthMap::DepthMapContainer>(&depths[0], rows, cols);

	dm.min_depth(min);
	dm.max_depth(max);
	
	dm.map_type = coarse ? RawDepthMap::MapType::COARSE : RawDepthMap::MapType::DENSE;
	dm.depth_type = metric ? RawDepthMap::DepthType::METRIC : RawDepthMap::DepthType::VIRTUAL;
}

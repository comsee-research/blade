#include "RawDepthMap.h"

//******************************************************************************
// constructors
//******************************************************************************
RawDepthMap::RawDepthMap(
	const PlenopticCamera& pcm, double mind, double maxd,
	DepthType dtype, MapType mtype
) : //map
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
}

RawDepthMap::RawDepthMap(const RawDepthMap& o)
:	//map
	map{o.height(), o.width()},
	//distances
	min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
	//depth map configuration
	depth_type{o.depth_type}, map_type{o.map_type}
{
	copy_from(o); //copy depth map data
}

RawDepthMap::RawDepthMap(RawDepthMap&& o)
:	//map
	map{std::move(o.map)},
	//distances
	min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
	//depth map configuration
	depth_type{o.depth_type}, map_type{o.map_type}
{
}

//******************************************************************************
// accessors
//******************************************************************************
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
void RawDepthMap::min_depth(double mind) { min_depth_ = mind;}
double RawDepthMap::max_depth() const { return max_depth_; }
void RawDepthMap::max_depth(double maxd) { max_depth_ = maxd; }

//******************************************************************************
// export functions
//******************************************************************************
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
				P2D idx;
				if (is_refined_map()) { const auto [k_, l_] =  pcm.mia().uv2kl(k, l); idx = pcm.mi2ml(k_, l_); }
				else { idx = pcm.mi2ml(k, l); }
				
				mdm.depth(k,l) = pcm.v2obj(depth(k,l), idx(0), idx(1));
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
				P2D idx;
				if (is_refined_map()) { const auto [k_, l_] =  pcm.mia().uv2kl(k, l); idx = pcm.mi2ml(k_, l_); }
				else { idx = pcm.mi2ml(k, l); }
				
				vdm.depth(k,l) = pcm.obj2v(depth(k,l), idx(0), idx(1));
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
bool RawDepthMap::is_refined_map() const { return not is_coarse_map(); }

bool RawDepthMap::is_valid_depth(double d) const { return not(is_depth_out_of_bounds(d)); }

bool RawDepthMap::is_depth_out_of_bounds(double d) const 
{
	return (d >= max_depth() or d < min_depth());
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
	
	dm.map_type = coarse ? RawDepthMap::MapType::COARSE : RawDepthMap::MapType::REFINED;
	dm.depth_type = metric ? RawDepthMap::DepthType::METRIC : RawDepthMap::DepthType::VIRTUAL;
}

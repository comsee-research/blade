#include "filter.h"

#include <pleno/processing/tools/stats.h>

#include "neighbors.h"
//******************************************************************************
//******************************************************************************
RawCoarseDepthMap median_filter_depth(const RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap filtereddm{dm};
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.mia().width()-1-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.mia().height()-1-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			double sz = size;
			if (size == AUTOMATIC_FILTER_SIZE) sz = dm.depth(k,l);
			
			//get neighbors
		 	std::vector<IndexPair> neighs = neighbors(dm.mia(), k, l, sz, sz); 
			
			std::vector<double> depths; depths.reserve(neighs.size()+1);
			depths.emplace_back(dm.depth(k,l));
			
			for(auto&n : neighs) depths.emplace_back(dm.depth(n.k, n.l));
		
			filtereddm.depth(k,l) = median(depths);
		}
	}
	
	return filtereddm;	
}

void inplace_median_filter_depth(RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap temp = median_filter_depth(dm, size);
	temp.copy_map(dm);
}

//******************************************************************************
//******************************************************************************
RawCoarseDepthMap erosion_filter_depth(const RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap filtereddm{dm}; //depths are copied
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.mia().width()-1-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.mia().height()-1-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			if(dm.depth(k,l) == DepthInfo::NO_DEPTH)
			{
				//get neighbors
		 		std::vector<IndexPair> neighs;
		 		
		 		if (size == AUTOMATIC_FILTER_SIZE) neighs = inner_ring(dm.mia(), k, l);
		 		else neighs = neighbors(dm.mia(), k, l, size, size); 
		 		
		 		for(auto& n: neighs) filtereddm.depth(n.k, n.l) = DepthInfo::NO_DEPTH;
			}
		}
	}
	
	return filtereddm;	
}

void inplace_erosion_filter_depth(RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap temp = erosion_filter_depth(dm, size);
	temp.copy_map(dm);
} 

//******************************************************************************
//******************************************************************************
RawCoarseDepthMap minmax_filter_depth(const RawCoarseDepthMap& dm, double min, double max)
{
	
	RawCoarseDepthMap temp{dm};
	inplace_minmax_filter_depth(temp, min, max);
	
	return temp;
}

void inplace_minmax_filter_depth(RawCoarseDepthMap& dm, double min, double max)
{	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.mia().width()-1-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.mia().height()-1-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			const double d = dm.depth(k,l);
			if(d < min or d > max) dm.depth(k,l) = DepthInfo::NO_DEPTH;
		}
	}
}

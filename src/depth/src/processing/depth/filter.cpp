#include "filter.h"

#include <pleno/processing/tools/stats.h>

#include "neighbors.h"

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
			//get neighbors
		 	std::vector<IndexPair> neighs = neighbors(dm.mia(), k, l, size, size); 
			
			std::vector<double> depths; depths.reserve(neighs.size()+1);
			depths.emplace_back(dm.depth(k,l));
			
			for(auto&n : neighs) depths.emplace_back(dm.depth(n.k, n.l));
		
			filtereddm.depth(k,l) = median(depths);
		}
	}
	
	return filtereddm;	
}

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
		 		std::vector<IndexPair> neighs = neighbors(dm.mia(), k, l, size, size); 
		 		for(auto& n: neighs) filtereddm.depth(n.k, n.l) = DepthInfo::NO_DEPTH;
			}
		}
	}
	
	return filtereddm;	
}

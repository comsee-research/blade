#include "neighbors.h"

#include <utility>


std::vector<IndexPair> inner_ring(const MIA& mia, std::size_t k, std::size_t l)
{
	constexpr std::size_t margin = 2;
	std::vector<IndexPair> indexes; indexes.reserve(6);
	
	const std::size_t kmax = mia.width()-1-margin; const std::size_t kmin = 0+margin;
	const std::size_t lmax = mia.height()-1-margin; const std::size_t lmin = 0+margin;
	
	for(auto [nk, nl] : std::vector<std::pair<int, int>>{{-1, 0}, {0, -1}, {0, 1}, {-1, 1}, {1, 0}, {1, 1}})
	{
		nk += k; nl += l;
		if(nk > kmax or nk < kmin or nl > lmax or nl < lmin) continue;
		
		indexes.emplace_back(
			static_cast<std::size_t>(nk), static_cast<std::size_t>(nl)
		);	
	}
	
	indexes.shrink_to_fit();
	return indexes;
}

std::vector<IndexPair> neighbors(const MIA& mia, std::size_t k, std::size_t l, double v, double innerring, double outterring)
{
	constexpr std::size_t margin = 2;
	std::vector<IndexPair> indexes;
	
	const std::size_t kmax = mia.width()-1-margin; const std::size_t kmin = 0+margin;
	const std::size_t lmax = mia.height()-1-margin; const std::size_t lmin = 0+margin;
	const double r = mia.radius() * std::min(std::max(std::fabs(v), innerring), outterring);
	
	for(int nk = k - r; nk < k + r; ++nk)
	{
		for(int nl = l - r; nl < l + r; ++nl)
		{
			if(nl == l and nk == k) continue; //same microimage
			if(nk > kmax or nk < kmin or nl > lmax or nl < lmin) continue; //out of indexes
			if((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l)).norm() > r) continue; //out of distance
			
			indexes.emplace_back(static_cast<std::size_t>(nk), static_cast<std::size_t>(nl));	
		}
	}
	
	return indexes;
}


#include "neighbors.h"

//#include <utility>

std::vector<IndexPair> inner_ring(const MIA& mia, std::size_t k, std::size_t l)
{
	constexpr std::size_t margin = 2;
	std::vector<IndexPair> indexes; indexes.reserve(6);
	
	const std::size_t kmax = mia.width()-1-margin; const std::size_t kmin = 0+margin;
	const std::size_t lmax = mia.height()-1-margin; const std::size_t lmin = 0+margin;
	
	for (auto [nk, nl] : std::vector<std::pair<int, int>>{{-1, 0}, {0, -1}, {0, 1}, {-1, 1}, {1, 0}, {1, 1}})
	{
		nk += k; nl += l;
		if (nk > kmax or nk < kmin or nl > lmax or nl < lmin) continue;
		
		indexes.emplace_back(
			static_cast<std::size_t>(nk), static_cast<std::size_t>(nl)
		);	
	}
	
	indexes.shrink_to_fit();
	return indexes;
}

std::vector<IndexPair> neighbors(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv, double maxv
)
{
	constexpr std::size_t margin = 2;
	std::vector<IndexPair> indexes;
	
	const std::size_t kmax = mia.width()-1-margin; const std::size_t kmin = 0+margin;
	const std::size_t lmax = mia.height()-1-margin; const std::size_t lmin = 0+margin;
	const double r = mia.radius() * std::min(std::max(std::fabs(v), minv), maxv) * 1.01;
	
	for (int nk = std::floor(k - r); nk < std::ceil(k + r); ++nk)
	{
		for (int nl = std::floor(l - r); nl < std::ceil(l + r); ++nl)
		{
			if (nl == l and nk == k) continue; //same microimage
			if (nk > kmax or nk < kmin or nl > lmax or nl < lmin) continue; //out of indexes
			if ((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l)).norm() > r) continue; //out of distance
			
			indexes.emplace_back(static_cast<std::size_t>(nk), static_cast<std::size_t>(nl));	
		}
	}
	
	return indexes;
}


std::map<double, std::vector<IndexPair>> neighbors_by_rings(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv, double maxv
)
{
	constexpr std::size_t margin = 2;
	std::map<double, std::vector<IndexPair>> indexes;
	
	const std::size_t kmax = mia.width()-1-margin; const std::size_t kmin = 0+margin;
	const std::size_t lmax = mia.height()-1-margin; const std::size_t lmin = 0+margin;
	const double r = mia.radius() * std::min(std::max(std::fabs(v), minv), maxv) * 1.01;
	
	for (int nk = std::floor(k - r); nk < std::ceil(k + r); ++nk)
	{
		for (int nl = std::floor(l - r); nl < std::ceil(l + r); ++nl)
		{
			if (nl == l and nk == k) continue; //same microimage
			if (nk > kmax or nk < kmin or nl > lmax or nl < lmin) continue; //out of indexes
			if ((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l)).norm() > r) continue; //out of distance
			
			const double d 	 	= (mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l)).norm();
			const double rd  	= std::ceil(10. * d - 0.5);
			const double dist 	= std::ceil(rd / mia.diameter() - 0.5) / 10.; 
			
			indexes[dist].emplace_back(static_cast<std::size_t>(nk), static_cast<std::size_t>(nl));	
		}
	}
	
	return indexes;
}

#if 0

std::vector<IndexPair> half_neighbors(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv, double maxv
)
{
	constexpr std::size_t margin = 2;
	std::vector<IndexPair> indexes;
	
	const std::size_t kmax = mia.width()-1-margin; const std::size_t kmin = 0+margin;
	const std::size_t lmax = mia.height()-1-margin; const std::size_t lmin = 0+margin;
	const double r = mia.radius() * std::min(std::max(std::fabs(v), minv), maxv) * 1.01;
	
	for (int nk = k; nk < std::ceil(k + r); ++nk)
	{
		for (int nl = std::floor(l - r); nl < std::ceil(l + r); ++nl)
		{
			if (nl == l and nk == k) continue; //same microimage
			if (nk > kmax or nk < kmin or nl > lmax or nl < lmin) continue; //out of indexes
			if ((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l)).norm() > r) continue; //out of distance
			if ((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l))[0] > 0.) continue; //out of distance
			
			indexes.emplace_back(static_cast<std::size_t>(nk), static_cast<std::size_t>(nl));	
		}
	}
	
	return indexes;
}

std::map<double, std::vector<IndexPair>> half_neighbors_by_rings(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv, double maxv
)
{
	constexpr std::size_t margin = 2;
	std::map<double, std::vector<IndexPair>> indexes;
	
	const std::size_t kmax = mia.width()-1-margin; const std::size_t kmin = 0+margin;
	const std::size_t lmax = mia.height()-1-margin; const std::size_t lmin = 0+margin;
	const double r = mia.radius() * std::min(std::max(std::fabs(v), minv), maxv) * 1.01;
	
	for (int nk = k; nk < std::ceil(k + r); ++nk)
	{
		for (int nl = std::floor(l - r); nl < std::ceil(l + r); ++nl)
		{
			if (nl == l and nk == k) continue; //same microimage
			if (nk > kmax or nk < kmin or nl > lmax or nl < lmin) continue; //out of indexes
			if ((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l)).norm() > r) continue; //out of distance
			if ((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l))[0] > 0.) continue; //out of distance
			
			const double d 	 	= (mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l)).norm();
			const double rd  	= std::ceil(10. * d - 0.5);
			const double dist 	= std::ceil(rd / mia.diameter() - 0.5) / 10.; 
			
			//indexes.try_emplace(dist, std::vector<IndexPair>{});
			indexes[dist].emplace_back(static_cast<std::size_t>(nk), static_cast<std::size_t>(nl));	
		}
	}
	
	return indexes;
}
#endif

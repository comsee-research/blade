#include "neighbors.h"

#include <pleno/io/printer.h>

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
	const double dv = std::min(std::max(std::fabs(v), minv), maxv) / 2.;
	const double r = mia.diameter() * dv * 1.01;
	
	for (int nk = std::floor(k - dv); nk < std::ceil(k + dv); ++nk)
	{
		for (int nl = std::floor(l - dv); nl < std::ceil(l + dv); ++nl)
		{
			if (nl == l and nk == k) continue; //same microimage
			if (nk > kmax or nk < kmin or nl > lmax or nl < lmin) continue; //out of indexes
			if ((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l)).norm() > r) continue; //out of distance
			
			indexes.emplace_back(static_cast<std::size_t>(nk), static_cast<std::size_t>(nl));	
		}
	}
	
	return indexes;
}

std::vector<IndexPair> pixels_neighbors(
	const MIA& mia, std::size_t W, std::size_t H, std::size_t k, std::size_t l
)	
{	
	const auto c = mia.nodeInWorld(k,l);
	const auto r = mia.radius() - 3. * mia.border();
	
	std::vector<IndexPair> indexes;
	
	const int umin = std::max(static_cast<int>(c[0] - r), 0);
	const int umax = std::min(static_cast<int>(W), static_cast<int>(c[0] + r));
	const int vmin = std::max(static_cast<int>(c[1] - r), 0);
	const int vmax = std::min(static_cast<int>(H), static_cast<int>(c[1] + r));
	
	for (int u = umin; u < umax; ++u)
	{
		for (int v = vmin; v < vmax; ++v)
		{
			const P2D p = {u, v};
			if ((p - c).norm() > r) continue; //out of distance
			
			indexes.emplace_back(static_cast<std::size_t>(u), static_cast<std::size_t>(v));	
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
	const double dv = std::min(std::max(std::fabs(v), minv), maxv) / 2.;
	const double r = mia.diameter() * dv * 1.01;
	
	for (int nk = std::floor(k - dv); nk < std::ceil(k + dv); ++nk)
	{
		for (int nl = std::floor(l - dv); nl < std::ceil(l + dv); ++nl)
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

#if 1

std::vector<IndexPair> half_neighbors(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv, double maxv
)
{
	constexpr std::size_t margin = 2;
	std::vector<IndexPair> indexes;
	
	const std::size_t kmax = mia.width()-1-margin; const std::size_t kmin = 0+margin;
	const std::size_t lmax = mia.height()-1-margin; const std::size_t lmin = 0+margin;
	
	const double dv = std::min(std::max(std::fabs(v), minv), maxv) / 2.;
	const double r = mia.diameter() * dv * 1.01;
	
	for (int nk = k; nk < std::ceil(k + dv); ++nk)
	{
		for (int nl = std::floor(l - dv); nl < std::ceil(l + dv); ++nl)
		{
			if (nl == l and nk == k) continue; //same microimage
			if (nk > kmax or nk < kmin or nl > lmax or nl < lmin) continue; //out of indexes
			if ((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l)).norm() > r) continue; //out of distance
			//if ((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l))[0] > 0.) continue; //out of distance
			
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
	const double dv = std::min(std::max(std::fabs(v), minv), maxv) / 2.;
	const double r = mia.diameter() * dv * 1.01;
	
	for (int nk = k; nk < std::ceil(k + dv); ++nk)
	{
		for (int nl = std::floor(l - dv); nl < std::ceil(l + dv); ++nl)
		{
			if (nl == l and nk == k) continue; //same microimage
			if (nk > kmax or nk < kmin or nl > lmax or nl < lmin) continue; //out of indexes
			if ((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l)).norm() > r) continue; //out of distance
			//if ((mia.nodeInWorld(nk, nl) - mia.nodeInWorld(k,l))[0] > 0.) continue; //out of distance
			
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

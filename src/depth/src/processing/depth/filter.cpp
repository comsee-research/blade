#include "filter.h"

#include <pleno/processing/tools/stats.h>

#include "neighbors.h"
//******************************************************************************
//******************************************************************************
RawDepthMap median_filter_depth(const RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	RawDepthMap filtereddm{dm};
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			double sz = size;
			if (size == AUTOMATIC_FILTER_SIZE) sz = dm.depth(k,l);
			
			//get neighbors
		 	std::vector<IndexPair> neighs = neighbors(mia, k, l, sz, sz); 
			
			std::vector<double> depths; depths.reserve(neighs.size()+1);
			depths.emplace_back(dm.depth(k,l));
			
			for(auto&n : neighs) depths.emplace_back(dm.depth(n.k, n.l));
		
			filtereddm.depth(k,l) = median(depths);
		}
	}
	
	return filtereddm;	
}
void inplace_median_filter_depth(RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const RawDepthMap temp = median_filter_depth(dm, mia, size);
	temp.copy_to(dm);
} 

//******************************************************************************
RawDepthMap mean_filter_depth(const RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	RawDepthMap filtereddm{dm};
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			double sz = size;
			if (size == AUTOMATIC_FILTER_SIZE) sz = dm.depth(k,l);
			
			//get neighbors
		 	std::vector<IndexPair> neighs = neighbors(mia, k, l, sz, sz); 
			
			std::vector<double> depths; depths.reserve(neighs.size()+1);
			depths.emplace_back(dm.depth(k,l));
			
			for(auto&n : neighs) depths.emplace_back(dm.depth(n.k, n.l));
		
			filtereddm.depth(k,l) = mean(depths);
		}
	}
	
	return filtereddm;	
}
void inplace_mean_filter_depth(RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const RawDepthMap temp = mean_filter_depth(dm, mia, size);
	temp.copy_to(dm);
} 

//******************************************************************************
RawDepthMap minmax_filter_depth(const RawDepthMap& dm, double min, double max)
{	
	RawDepthMap temp{dm};
	inplace_minmax_filter_depth(temp, min, max);
	
	return temp;
}

void inplace_minmax_filter_depth(RawDepthMap& dm, double min, double max)
{		
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
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


//******************************************************************************
//******************************************************************************
RawDepthMap erosion_filter_depth(const RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	RawDepthMap filtereddm{dm}; //depths are copied
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			if(dm.depth(k,l) == DepthInfo::NO_DEPTH)
			{
				//get neighbors
		 		std::vector<IndexPair> neighs;
		 		
		 		if (size == AUTOMATIC_FILTER_SIZE) neighs = inner_ring(mia, k, l);
		 		else neighs = neighbors(mia, k, l, size, size); 
		 		
		 		for(auto& n: neighs) filtereddm.depth(n.k, n.l) = DepthInfo::NO_DEPTH;
			}
		}
	}
	
	return filtereddm;	
}
void inplace_erosion_filter_depth(RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const RawDepthMap temp = erosion_filter_depth(dm, mia, size);
	temp.copy_to(dm);
} 

//******************************************************************************
//******************************************************************************
/* E(I,Z)= min I(Z) */ 
RawDepthMap morph_erosion_filter_depth(const RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	RawDepthMap filtereddm{dm}; //depths are copied
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			//get neighbors
	 		std::vector<IndexPair> neighs;
	 		
	 		if (size == AUTOMATIC_FILTER_SIZE) neighs = inner_ring(mia, k, l);
	 		else neighs = neighbors(mia, k, l, size, size); 
	 		
	 		for (auto& n: neighs)
	 		{
	 			if (filtereddm.depth(k,l) > dm.depth(n.k,n.l))
	 			{
	 				filtereddm.depth(k,l) = dm.depth(n.k,n.l);
	 			}
	 		}
		}
	}
	
	return filtereddm;	
}
void inplace_morph_erosion_filter_depth(RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const RawDepthMap temp = morph_erosion_filter_depth(dm, mia, size);
	temp.copy_to(dm);
} 

//******************************************************************************
/* D(I,Z)= max I(Z) */ 
RawDepthMap morph_dilation_filter_depth(const RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	RawDepthMap filtereddm{dm}; //depths are copied
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			//get neighbors
	 		std::vector<IndexPair> neighs;
	 		
	 		if (size == AUTOMATIC_FILTER_SIZE) neighs = inner_ring(mia, k, l);
	 		else neighs = neighbors(mia, k, l, size, size); 
	 		
	 		for (auto& n: neighs)
	 		{
	 			if (filtereddm.depth(k,l) < dm.depth(n.k,n.l))
	 			{
	 				filtereddm.depth(k,l) = dm.depth(n.k,n.l);
	 			}
	 		}
		}
	}
	
	return filtereddm;	
}
void inplace_morph_dilation_filter_depth(RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const RawDepthMap temp = morph_dilation_filter_depth(dm, mia, size);
	temp.copy_to(dm);
} 

//******************************************************************************
/* O(I,Z)= D(E(I,Z),Z) */ 
RawDepthMap morph_opening_filter_depth(const RawDepthMap& dm, const MIA& mia, double size)
{	
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	RawDepthMap filtereddm{dm};
	inplace_morph_opening_filter_depth(filtereddm, mia, size);
	
	return filtereddm;
} 
void inplace_morph_opening_filter_depth(RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	inplace_erosion_filter_depth(dm, mia, size);
	inplace_morph_dilation_filter_depth(dm, mia, size);
} 

//******************************************************************************
/* C(I,Z)= E(D(I,Z),Z) */ 
RawDepthMap morph_closing_filter_depth(const RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	RawDepthMap filtereddm{dm};
	inplace_morph_closing_filter_depth(filtereddm, mia, size);
	
	return filtereddm;
} 
void inplace_morph_closing_filter_depth(RawDepthMap& dm, const MIA& mia, double size)
{	
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	inplace_morph_dilation_filter_depth(dm, mia, size);
	inplace_erosion_filter_depth(dm, mia, size);
} 

//******************************************************************************
/* S(I,Z)= C(O(I,Z),Z) */ 
RawDepthMap morph_smoothing_filter_depth(const RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");

	RawDepthMap filtereddm{dm};
	inplace_morph_smoothing_filter_depth(filtereddm, mia, size);
	
	return filtereddm;
} 
void inplace_morph_smoothing_filter_depth(RawDepthMap& dm, const MIA& mia, double size)
{	
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	inplace_morph_opening_filter_depth(dm, mia, size);
	inplace_morph_closing_filter_depth(dm, mia, size);
} 

//******************************************************************************
/* DYT(I,Z) = 0.5 * (E(I,Z) + D(I,Z)) */
RawDepthMap morph_dyt_filter_depth(const RawDepthMap& dm, const MIA& mia, double size)
{	
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	RawDepthMap filtereddm{dm};
	inplace_morph_dyt_filter_depth(filtereddm, mia, size);
	
	return filtereddm;
} 
void inplace_morph_dyt_filter_depth(RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const RawDepthMap edm = morph_erosion_filter_depth(dm, mia, size);
	const RawDepthMap ddm = morph_dilation_filter_depth(dm, mia, size);
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			dm.depth(k,l) = 0.5 * (edm.depth(k,l) + ddm.depth(k,l));
		}
	}
} 

//******************************************************************************
/* TET(I,Z) = 0.5 * (O(I,Z) + C(I,Z)) */
RawDepthMap morph_tet_filter_depth(const RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	RawDepthMap filtereddm{dm};
	inplace_morph_tet_filter_depth(filtereddm, mia, size);
	
	return filtereddm;	
} 
void inplace_morph_tet_filter_depth(RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const RawDepthMap odm = morph_opening_filter_depth(dm, mia, size);
	const RawDepthMap cdm = morph_closing_filter_depth(dm, mia, size);
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			dm.depth(k,l) = 0.5 * (odm.depth(k,l) + cdm.depth(k,l));
		}
	}
} 

//******************************************************************************
/* OCCO(I,Z) = 0.5 * (O(C(I,Z),Z) + C(O(I,Z),Z)) */
RawDepthMap morph_occo_filter_depth(const RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	RawDepthMap filtereddm{dm}; //depths are copied
	inplace_morph_occo_filter_depth(filtereddm, mia, size);
	
	return filtereddm;	
} 
void inplace_morph_occo_filter_depth(RawDepthMap& dm, const MIA& mia, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	RawDepthMap ocdm = morph_closing_filter_depth(dm, mia, size);
	inplace_morph_opening_filter_depth(ocdm, mia, size);
	
	RawDepthMap codm = morph_opening_filter_depth(dm, mia, size);
	inplace_morph_closing_filter_depth(codm, mia, size);
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			dm.depth(k,l) = 0.5 * (ocdm.depth(k,l) + codm.depth(k,l));
		}
	}
} 
//******************************************************************************
//******************************************************************************

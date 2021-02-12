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
	const RawCoarseDepthMap temp = median_filter_depth(dm, size);
	dm.copy_map(temp);
} 

//******************************************************************************
RawCoarseDepthMap mean_filter_depth(const RawCoarseDepthMap& dm, double size)
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
		
			filtereddm.depth(k,l) = mean(depths);
		}
	}
	
	return filtereddm;	
}
void inplace_mean_filter_depth(RawCoarseDepthMap& dm, double size)
{
	const RawCoarseDepthMap temp = mean_filter_depth(dm, size);
	dm.copy_map(temp);
} 

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
	const RawCoarseDepthMap temp = erosion_filter_depth(dm, size);
	dm.copy_map(temp);
} 

//******************************************************************************
//******************************************************************************
/* E(I,Z)= min I(Z) */ 
RawCoarseDepthMap morph_erosion_filter_depth(const RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap filtereddm{dm}; //depths are copied
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.mia().width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.mia().height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			//get neighbors
	 		std::vector<IndexPair> neighs;
	 		
	 		if (size == AUTOMATIC_FILTER_SIZE) neighs = inner_ring(dm.mia(), k, l);
	 		else neighs = neighbors(dm.mia(), k, l, size, size); 
	 		
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
void inplace_morph_erosion_filter_depth(RawCoarseDepthMap& dm, double size)
{
	const RawCoarseDepthMap temp = morph_erosion_filter_depth(dm, size);
	dm.copy_map(temp);
} 

//******************************************************************************
/* D(I,Z)= max I(Z) */ 
RawCoarseDepthMap morph_dilation_filter_depth(const RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap filtereddm{dm}; //depths are copied
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.mia().width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.mia().height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			//get neighbors
	 		std::vector<IndexPair> neighs;
	 		
	 		if (size == AUTOMATIC_FILTER_SIZE) neighs = inner_ring(dm.mia(), k, l);
	 		else neighs = neighbors(dm.mia(), k, l, size, size); 
	 		
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
void inplace_morph_dilation_filter_depth(RawCoarseDepthMap& dm, double size)
{
	const RawCoarseDepthMap temp = morph_dilation_filter_depth(dm, size);
	dm.copy_map(temp);
} 

//******************************************************************************
/* O(I,Z)= D(E(I,Z),Z) */ 
RawCoarseDepthMap morph_opening_filter_depth(const RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap filtereddm{dm};
	inplace_morph_opening_filter_depth(filtereddm, size);
	
	return filtereddm;
} 
void inplace_morph_opening_filter_depth(RawCoarseDepthMap& dm, double size)
{
	inplace_erosion_filter_depth(dm, size);
	inplace_morph_dilation_filter_depth(dm, size);
} 

//******************************************************************************
/* C(I,Z)= E(D(I,Z),Z) */ 
RawCoarseDepthMap morph_closing_filter_depth(const RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap filtereddm{dm};
	inplace_morph_closing_filter_depth(filtereddm, size);
	
	return filtereddm;
} 
void inplace_morph_closing_filter_depth(RawCoarseDepthMap& dm, double size)
{
	inplace_morph_dilation_filter_depth(dm, size);
	inplace_erosion_filter_depth(dm, size);
} 

//******************************************************************************
/* S(I,Z)= C(O(I,Z),Z) */ 
RawCoarseDepthMap morph_smoothing_filter_depth(const RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap filtereddm{dm};
	inplace_morph_smoothing_filter_depth(filtereddm, size);
	
	return filtereddm;
} 
void inplace_morph_smoothing_filter_depth(RawCoarseDepthMap& dm, double size)
{
	inplace_morph_opening_filter_depth(dm, size);
	inplace_morph_closing_filter_depth(dm, size);
} 

//******************************************************************************
/* DYT(I,Z) = 0.5 * (E(I,Z) + D(I,Z)) */
RawCoarseDepthMap morph_dyt_filter_depth(const RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap filtereddm{dm};
	inplace_morph_dyt_filter_depth(filtereddm, size);
	
	return filtereddm;
} 
void inplace_morph_dyt_filter_depth(RawCoarseDepthMap& dm, double size)
{
	const RawCoarseDepthMap edm = morph_erosion_filter_depth(dm, size);
	const RawCoarseDepthMap ddm = morph_dilation_filter_depth(dm, size);
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.mia().width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.mia().height()-margin; 
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
RawCoarseDepthMap morph_tet_filter_depth(const RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap filtereddm{dm};
	inplace_morph_tet_filter_depth(filtereddm, size);
	
	return filtereddm;	
} 
void inplace_morph_tet_filter_depth(RawCoarseDepthMap& dm, double size)
{
	const RawCoarseDepthMap odm = morph_opening_filter_depth(dm, size);
	const RawCoarseDepthMap cdm = morph_closing_filter_depth(dm, size);
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.mia().width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.mia().height()-margin; 
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
RawCoarseDepthMap morph_occo_filter_depth(const RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap filtereddm{dm}; //depths are copied
	inplace_morph_occo_filter_depth(filtereddm, size);
	
	return filtereddm;	
} 
void inplace_morph_occo_filter_depth(RawCoarseDepthMap& dm, double size)
{
	RawCoarseDepthMap ocdm = morph_closing_filter_depth(dm, size);
	inplace_morph_opening_filter_depth(ocdm, size);
	
	RawCoarseDepthMap codm = morph_opening_filter_depth(dm, size);
	inplace_morph_closing_filter_depth(codm, size);
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.mia().width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.mia().height()-margin; 
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

#pragma once

#include <vector>
#include <map>


#include <pleno/geometry/mia.h>

#include "../../types.h"

std::vector<IndexPair> 
inner_ring(const MIA& mia, std::size_t k, std::size_t l);

std::vector<IndexPair> neighbors(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv = 5., double maxv = 12.
);

std::vector<IndexPair> half_neighbors(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv = 5., double maxv = 12.
);

std::map<double, std::vector<IndexPair>> neighbors_by_rings(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv = 5., double maxv = 12.
);

std::map<double, std::vector<IndexPair>> half_neighbors_by_rings(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv = 5., double maxv = 12.
);

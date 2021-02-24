#pragma once

#include <pleno/types.h>

struct IndexPair {
	std::size_t k, l;
	
	IndexPair(std::size_t k_ = 0u, std::size_t l_ = 0u): k{k_}, l{l_} {}
};

struct xyz 
{
	double x = 0., y = 0., z = 0.;
};

using XYZs = std::vector<xyz>;


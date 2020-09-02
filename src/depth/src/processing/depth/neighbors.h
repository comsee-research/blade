#pragma once

#include <vector>

#include <pleno/geometry/mia.h>

#include "../../types.h"

std::vector<IndexPair> inner_ring(const MIA& mia, std::size_t k, std::size_t l);
std::vector<IndexPair> neighbors(const MIA& mia, std::size_t k, std::size_t l, double v, double innerring = 5., double outterring = 12.);

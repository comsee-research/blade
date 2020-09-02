#pragma once

#include <random>

#include <pleno/types.h>

#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/mia.h>

#include "../../types.h"
#include "strategy.h"

IndexPair initialize_kl(
	std::size_t i, std::size_t n, 
	const MIA& mia, 
	InitStrategy mode = InitStrategy::REGULAR_GRID
);

double initialize_depth(
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t ck, std::size_t cl,
	double minv = 2., double maxv = 20., double nbsample = 10.,
	ObservationsParingStrategy mode = ObservationsParingStrategy::CENTRALIZED
);

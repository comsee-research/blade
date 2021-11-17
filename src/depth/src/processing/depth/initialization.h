#pragma once

#include <pleno/types.h>

#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/mia.h>

#include <pleno/processing/depth/initialization.h>

#include "../../types.h"

#include "depth.h"
#include "strategy.h"

IndexPair initialize_kl(
	std::size_t i, std::size_t n, 
	const MIA& mia, 
	InitStrategy mode = InitStrategy::REGULAR_GRID
);

void initialize_depth(
	DepthHypothesis& hypothesis,
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const DepthEstimationStrategy& strategies
);

#pragma once

#include <pleno/types.h>
#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/observation.h>

#include "../../types.h"
#include "strategy.h"

#include "geometry/depth/RawCoarseDepthMap.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap(
	RawCoarseDepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	DepthEstimationStrategy strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_probabilistic_depthmap(
	RawCoarseDepthMap& dm,	RawCoarseDepthMap& confidencedm,
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	DepthEstimationStrategy strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap_from_obs(
	RawCoarseDepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
);

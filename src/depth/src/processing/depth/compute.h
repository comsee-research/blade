#pragma once

#include <pleno/types.h>
#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/observation.h>

#include "../../types.h"
#include "strategy.h"

#include "geometry/depth/depthmap.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap(
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_refined_depthmap(
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_probabilistic_depthmap(
	DepthMap& dm,
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap_from_obs(
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
);

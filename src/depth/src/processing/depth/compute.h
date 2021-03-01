#pragma once

#include <pleno/types.h>
#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/observation.h>

#include "../../types.h"
#include "strategy.h"

#include "geometry/depth/RawDepthMap.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap(
	RawDepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_dense_depthmap(
	RawDepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_probabilistic_depthmap(
	RawDepthMap& dm,
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap_from_obs(
	RawDepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
);

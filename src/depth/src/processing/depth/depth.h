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
void estimate_depth(
	RawCoarseDepthMap& dm,
	const PlenopticCamera& mfpc,
	const Image& img,
	const DepthEstimationStrategy& strategies
);

void estimate_probabilistic_depth(
	RawCoarseDepthMap& dm, RawCoarseDepthMap& confidencedm,
	const PlenopticCamera& mfpc,
	const Image& img,
	const DepthEstimationStrategy& strategies
);

void estimate_depth_from_obs(
	RawCoarseDepthMap& rcdm,
	const PlenopticCamera& mfpc,
	const Image& img,
	const BAPObservations& observations /*  (u,v,rho) */
);


#pragma once

#include <pleno/types.h>
#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/observation.h>

void estimate_depth_from_observations(
	const PlenopticCamera& mfpc,    
	const BAPObservations& observations, /*  (u,v,rho) */
	const std::vector<Image>& images
);

void estimate_depth(
	const PlenopticCamera& mfpc,
	const Image& img
);


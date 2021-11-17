#pragma once

#include <pleno/types.h>
#include <pleno/geometry/camera/plenoptic.h>

#include "geometry/depth/depthmap.h"

#include "strategy.h"

void export_cost_function_from_obs(
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations,
	double minv = 2., double maxv = 20., double nbsample = 10.	
);

void export_cost_function(
	const PlenopticCamera& mfpc, const Image& scene, 
	double minv = 2., double maxv = 20., double nbsample = 10.,
	ObservationsPairingStrategy mode = ObservationsPairingStrategy::CENTRALIZED,
	bool export_per_baseline = false
);

void export_depth_histogram(
	const DepthMap& dm, bool lighten = true
);

void export_micro_images_blade(
	const PlenopticCamera& mfpc, const Image& scene
);

//--------- helper
IndexPair extract_micro_image_indexes(
	const Image& gray, const MIA& grid
);

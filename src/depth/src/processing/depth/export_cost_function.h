#pragma once

#include <pleno/types.h>
#include <pleno/geometry/camera/plenoptic.h>

#include "strategy.h"

void export_cost_function(
	const PlenopticCamera& mfpc, const Image& scene, 
	double minv = 2., double maxv = 20., double nbsample = 10.,
	ObservationsParingStrategy mode = ObservationsParingStrategy::CENTRALIZED
);

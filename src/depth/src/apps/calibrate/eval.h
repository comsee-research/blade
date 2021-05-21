#pragma once

#include <unordered_map>

#include <pleno/types.h>

//geometry
#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/object/checkerboard.h>
#include <pleno/geometry/observation.h>
#include "geometry/depth/RawDepthMap.h"

#include "processing/tools/functions.h"

void evaluate_scale_error(
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, RawDepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations,
	const std::unordered_map<Index, Image>& pictures
);

void evaluate_scale_error(
	const PlenopticCamera& mfpc, const LinearFunction& scaling,
	const CheckerBoard& scene,
	const std::unordered_map<Index, RawDepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations,
	const std::unordered_map<Index, Image>& pictures
);

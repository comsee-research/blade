#pragma once

#include <type_traits>

#include <pleno/processing/calibration/calibration.h>

#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/object/checkerboard.h>
#include <pleno/geometry/observation.h>

#include "geometry/depth/RawDepthMap.h"

#include <pleno/processing/tools/functions.h>

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_depthScaling(                        
	auto& scaling,  
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, RawDepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
);

#include "scaling.hpp"



#pragma once

#include <type_traits>

#include <pleno/processing/calibration/calibration.h>

#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/object/checkerboard.h>
#include <pleno/geometry/object/constellation.h>
#include <pleno/geometry/observation.h>

#include "geometry/depth/RawDepthMap.h"

#include <pleno/processing/tools/functions.h>


//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_LidarPlenopticCamera(                        
	CalibrationPose& pose, /* out */                   
	const PlenopticCamera& model, /* in */   
	const PointsConstellation& constellation,
	const BAPObservations& observations, /* (u,v,rho?) */
	const Image& scene
);

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



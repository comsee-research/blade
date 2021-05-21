#pragma once

#include <libv/lma/lma.hpp>

#include <map>

#include <pleno/types.h>
#include "../../types.h"

#include <pleno/geometry/camera/plenoptic.h> //PlenopticCamera
#include <pleno/geometry/object/checkerboard.h>
#include <pleno/geometry/observation.h>

#include "geometry/depth/RawDepthMap.h"

#include <pleno/processing/tools/functions.h>

struct ScalingCostError
{		
	using ErrorType = Eigen::Matrix<double, 1, 1>; //mae
	
	const PlenopticCamera& mfpc;
	const CheckerBoard& scene;
	
	const RawDepthMap& dm;
	std::unordered_map<Index /* cluster index */, BAPObservations> clusters;
	
    ScalingCostError(
    	const PlenopticCamera& mfpc_,
    	const CheckerBoard& scene_, 
    	const RawDepthMap& dm_,
    	const BAPObservations& observations
    );

    bool operator()( 
    	const LinearFunction& f,
    	ErrorType& error
    ) const;
};
namespace ttt
{
	template<> 
	struct Name<ScalingCostError> 
	{ 
		static std::string name(){ return "ScalingCostError"; } 
	};
} // namespace ttt

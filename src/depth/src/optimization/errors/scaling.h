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

template <typename FunctionType>
struct ScalingCostError
{		
	using ErrorType = Eigen::Matrix<double, 1, 1>; //mbe
	
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
    	const FunctionType& f,
    	ErrorType& error
    ) const;
};

using LinearScalingCostError 	= ScalingCostError<LinearFunction>;
using QuadraticScalingCostError = ScalingCostError<QuadraticFunction>;

namespace ttt
{
	template<> 
	struct Name<LinearScalingCostError> 
	{ 
		static std::string name(){ return "LinearScalingCostError"; } 
	};
	
	template<> 
	struct Name<QuadraticScalingCostError> 
	{ 
		static std::string name(){ return "QuadraticScalingCostError"; } 
	};
} // namespace ttt

#include "calibration.h"

//optimization
#include "optimization/functions.h"
#include "optimization/errors/scaling.h" //ScalingCostError

//io
#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<>
void calibration_depthScaling<LinearFunction>(                        
	LinearFunction& scaling,  
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, RawDepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
);

template<>
void calibration_depthScaling<QuadraticFunction>(                        
	QuadraticFunction& scaling,
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, RawDepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
);
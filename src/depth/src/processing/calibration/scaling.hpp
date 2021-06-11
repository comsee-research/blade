#pragma once

#include <type_traits>

#include <pleno/processing/calibration/calibration.h>

#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/object/checkerboard.h>
#include <pleno/geometry/observation.h>

#include "geometry/depth/RawDepthMap.h"

#include <pleno/processing/tools/functions.h>

//optimization
#include "optimization/functions.h"
#include "optimization/errors/scaling.h" //ScalingCostError

//io
#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

void optimize(
	auto& scaling, //LinearFunction& scaling,    
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, RawDepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
)
{		
	using FunctionType = std::decay_t<decltype(scaling)>;	
	using Solver_t = lma::Solver<ScalingCostError<FunctionType>>;
	
	Solver_t solver{-1., 1500, 1.0 - 1e-19};

	//for each frame
	for (const auto& [frame, dm] : depthmaps)
	{
		if (auto it = observations.find(frame); it != observations.end())
		{
			const BAPObservations& bap = it->second;
			
			//add ref point to solver
			solver.add(
				ScalingCostError<FunctionType>{mfpc, scene, dm, bap},
				&scaling
			);
		}	
		else
		{
			PRINT_ERR("No functor to add for frame f = " << frame);
		}
	}
	
	solver.solve(lma::DENSE, lma::enable_verbose_output());
}


void calibration_depthScaling(                        
	auto& scaling,     
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, RawDepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
)
{	
	using FunctionType = std::decay_t<decltype(scaling)>;	
//1) Init Parameters
	PRINT_INFO("=== Init Parameter");	
	FunctionType f;
	
//3) Run optimization
	PRINT_INFO("=== Run optimization");	
	optimize(f, mfpc, scene, depthmaps, observations);
	
	PRINT_INFO("=== Optimization finished! Results:");
	
	if constexpr (std::is_same_v<FunctionType, LinearFunction>)
	{
		PRINT_INFO("Scaling is f(z) = "<< f.a << " * z + " << f.b );
			
		scaling.a = f.a;
		scaling.b = f.b;
	} 
	else if constexpr (std::is_same_v<FunctionType, QuadraticFunction>)
	{	
		PRINT_INFO("Scaling is f(z) = "<< f.a << " * z² + " << f.b << " * z + " << f.c);
	
		scaling.a = f.a;
		scaling.b = f.b;
		scaling.c = f.c;
	}
	
	wait();
}

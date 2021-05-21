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
void optimize(
	LinearFunction& scaling,    
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, RawDepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
)
{			
	using Solver_t = lma::Solver<ScalingCostError>;
	
	Solver_t solver{-1., 1500, 1.0 - 1e-19};

	//for each frame
	for (const auto& [frame, dm] : depthmaps)
	{
		if (auto it = observations.find(frame); it != observations.end())
		{
			const BAPObservations& bap = it->second;
			
			//add ref point to solver
			solver.add(
				ScalingCostError{mfpc, scene, dm, bap},
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

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_depthScaling(                        
	LinearFunction& scaling, /* out */      
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, RawDepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
)
{
//1) Init Parameters
	PRINT_INFO("=== Init Parameter");	
	LinearFunction f{1., 0.};
	
//3) Run optimization
	PRINT_INFO("=== Run optimization");	
	optimize(f, mfpc, scene, depthmaps, observations);
	
	PRINT_INFO("=== Optimization finished! Results:");
	PRINT_INFO("Scaling is f(z) = "<< f.a << " * z + " << f.b );
	
	scaling.a = f.a;
	scaling.b = f.b;

	wait();
}

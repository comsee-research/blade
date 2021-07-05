#include "calibration.h"

#include <type_traits> // std::remove_reference_t
#include <variant> //std::variant

//optimization
#include "optimization/optimization.h"
#include "optimization/errors/lidarcamera.h" //LidarCameraCornerReprojectionError, LidarCameraBlurAwarePlenopticReprojectionError

//io
#include <pleno/io/cfg/observations.h> // save and load
#include <pleno/io/cfg/poses.h>

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

//graphic
#include <pleno/graphic/gui.h>
#include "graphic/display.h"

//calibration
#include "link.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize(
	//OUT
	CalibrationPose& transformation, /* extrinsics */
	//IN
	const PlenopticCamera& model, /* intrinsics */
	const PointsConstellation& constellation,
	const BAPObservations& observations /*  (u,v,rho) */
)
{
	const bool useRadius = (model.focused()); 
	
	using SolverBAP = lma::Solver<LidarCameraBlurAwarePlenopticReprojectionError>;
	using SolverCorner = lma::Solver<LidarCameraCornerReprojectionError>;
	using Solver_t = std::variant<std::monostate, SolverBAP, SolverCorner>;
	
	Solver_t vsolver;
		if(useRadius) vsolver.emplace<SolverBAP>(1e-4, 150, 1.0 - 1e-12);
		else vsolver.emplace<SolverCorner>(1e-4, 150, 1.0 - 1e-12);  

	Pose p = transformation.pose;	
	
	std::visit(
		[&](auto&& s) { 
			using T = std::decay_t<decltype(s)>;
			if constexpr (not std::is_same_v<T, std::monostate>) 
			{
				using Error_t = typename std::conditional<std::is_same_v<T, SolverBAP>, 
					LidarCameraBlurAwarePlenopticReprojectionError, 
					LidarCameraCornerReprojectionError
				>::type;
				
				for (const auto& o : observations) //for each observation
						s.add(Error_t{model, constellation, o}, &p);
						
				s.solve(lma::DENSE, lma::enable_verbose_output());
			}
		}, vsolver
	);
	
	transformation.pose = p;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_LidarPlenopticCamera(                        
	CalibrationPose& pose, /* in: initial pose, out: optimized pose */                   
	const PlenopticCamera& model, /* in */   
	const PointsConstellation& constellation,
	const BAPObservations& observations, /* (u,v,rho?) */
	const Image& scene
)
{	
	DEBUG_VAR(observations.size());
	
	BAPObservations features{observations.begin(), observations.end()};	
	DEBUG_VAR(features.size());

//1) Link Observations
	PRINT_INFO("=== Link Observations");	
	link_cluster_to_point_constellation_index(features, model, constellation, scene);
	DEBUG_VAR(features.size());
	
	PRINT_DEBUG("Remove not affected features");
	features.erase(
		std::remove_if(
			features.begin(), features.end(), 
			[max_id = constellation.size()](const auto& f){
				return (not f.isValid) or f.cluster == -1 or f.cluster >= int(max_id);
			}
		),
		features.end()
	);
	features.shrink_to_fit();	
	DEBUG_VAR(features.size());
	
	ObservationsConfig cfg_obs;
	cfg_obs.features() = features;
		
	v::save("linked-observations-"+std::to_string(getpid())+".bin.gz", cfg_obs);
	
	PRINT_DEBUG("Change indexes' space from MI to ML");
	model.mi2ml(features);
	
//2) Run optimization
	PRINT_INFO("=== Run optimization");
	const CalibrationPose porigin{Pose{}, -1}; 
	pose.frame = 0;
	display(porigin); display(constellation); display(pose);
	
	wait();
	optimize(pose, model, constellation, features);

	PRINT_DEBUG("Optimized pose:" << pose.pose);
	{
		CalibrationPoseConfig cfg_pose;
		cfg_pose.pose() = pose.pose;
		cfg_pose.frame() = pose.frame;
		
		v::save(
			"lidar-camera-pose-"+std::to_string(getpid())+".js", 
			cfg_pose
		);
	}		
	
	wait();
	//apply transformation on pointcloud and visually check the results
}

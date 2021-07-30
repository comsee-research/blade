#pragma once

#include <pleno/types.h>

#include <pleno/geometry/camera/models.h> //ThinLensCamera
#include <pleno/geometry/object/constellation.h> //PointsConstellation

#include <pleno/geometry/pose.h>

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

#include <pleno/graphic/gui.h>
#include "graphic/display.h"

#include <pleno/processing/tools/error.h>
#include <pleno/processing/algorithms/p3p.h>

template<typename CameraModel, typename Observations>
PoseWithError select_best_pose(
	const CameraModel& camera, 
	const PointsConstellation& scene,
    const Observations& observations, 
    const Poses& poses
);

template<typename CameraModel, typename Observations>
PoseWithError estimate_pose(
	const CameraModel& model, 
	const PointsConstellation& scene, 
	const Observations& barycenters
);

template<class CameraModel, class Observations>
void init_extrinsic(
	//OUT
	CalibrationPose& pose,
	//IN
	const CameraModel& model,
	const PointsConstellation& scene,
	const Observations& observations,
	//GUI
	const Image& picture = {} /* for GUI only */
);

#define INIT_AT_FOCAL_LENGTH 0

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename CameraModel, typename Observations>
PoseWithError select_best_pose(
	const CameraModel& camera, 
	const PointsConstellation& scene,
    const Observations& observations, 
    const Poses& poses
)
{       
    PosesWithError rmse_poses;
    rmse_poses.reserve(poses.size());

    // for each camera pose
    for (const auto& p : poses)
    {
    	PRINT_DEBUG("For pose p = " << p);
        RMSE rmse{0., 0};
             
    	PRINT_DEBUG("For each observation, compute rmse");
        for (const auto& o : observations)
        {
            if(not o.isValid) { rmse.add(1e10); continue; }
            
            const P3D p3d_cam = to_coordinate_system_of(p, scene.get(o.cluster));
            
            P2D prediction; // a projected chessboard node projected in IMAGE UV
            if (camera.project(p3d_cam, prediction))
            {
                rmse.add( P2D{P2D{o[0], o[1]} - prediction} );
            }
            else //projected outside the checkboard
            {
            	//PRINT_ERR("Node projected outside of sensor");
                rmse.add(1e10); // penalty
                break;
            }
        }
        
        if(p.translation().z() > 0.) rmse.add(1e20); // penalty if pose is on the otherside

        rmse_poses.emplace_back(PoseWithError{p, rmse});
    }
	
	PRINT_DEBUG("Find the best pose with the lowest rmse");
    // then sort the tuple according to rms
    std::sort(
    	rmse_poses.begin(), rmse_poses.end(),
		[](const auto& a, const auto& b) { 
              return a.rmse.get() < b.rmse.get(); 
        }
    );
    for(const auto&p : rmse_poses) PRINT_DEBUG("RMSE = " << p.rmse.get());
	
	PRINT_DEBUG("Best pose is p = " << rmse_poses[0].pose << "with rmse = " << rmse_poses[0].rmse.get());
   	return rmse_poses[0];
}

template<typename CameraModel, typename Observations>
PoseWithError estimate_pose(
	const CameraModel& model, 
	const PointsConstellation& scene, 
	const Observations& barycenters
)
{
	//Configure monocular camera 
	PRINT_DEBUG("Configure monocular camera ");
	Sensor film = Sensor{model.sensor().width(), model.sensor().height(), model.sensor().scale()};
	film.pose() = model.sensor().pose();
#if defined(INIT_AT_FOCAL_LENGTH) && INIT_AT_FOCAL_LENGTH
	film.pose().translation().z() = - model.focal();
#endif
	const PinholeCamera monocular{model.focal(), film};
	
	//Compute pose candidates using p3p
	PRINT_DEBUG("Compute pose candidates using p3p"); 
    std::array<Ray3D, 3> rays; // computing rays corresponding to each nodes
    {//0
		P2D pixel = P2D{barycenters[0].u, barycenters[0].v}; //in UV space	    
		monocular.raytrace(pixel, rays.at(0));
	}
	{//1
		P2D pixel = P2D{barycenters[1].u, barycenters[1].v}; //in UV space	    
		monocular.raytrace(pixel, rays.at(1));
	}	
	{//2
		P2D pixel = P2D{barycenters[2].u, barycenters[2].v}; //in UV space	     
		monocular.raytrace(pixel, rays.at(2));
	}	
    // computing p3p
    Poses candidates(4); 
    bool p3p_ok = solve_p3p(
    	scene.get(barycenters[0].cluster),
    	scene.get(barycenters[1].cluster),
    	scene.get(barycenters[2].cluster),
		rays.at(0).direction(), 
		rays.at(1).direction(),
		rays.at(2).direction(),
		candidates
	);

	DEBUG_VAR(p3p_ok);
	
	for (const auto &pose : candidates)
	{
		if ( not(((pose.translation().array() == pose.translation().array())).all()) //check is_nan
			or not(((pose.rotation().array() == pose.rotation().array())).all()) //check is_nan
		)
		{
			PRINT_ERR("Pose contains NaN");
			DEBUG_VAR(pose);
			DEBUG_VAR(barycenters[0]);
			DEBUG_VAR(barycenters[1]);
			DEBUG_VAR(barycenters[2]);
			DEBUG_VAR(rays.at(0));
			DEBUG_VAR(rays.at(1));
			DEBUG_VAR(rays.at(2));
		}
	}
	
	//Select best using RANSAC
	PRINT_DEBUG("Select best using RANSAC");
	PoseWithError extrinsics = select_best_pose(monocular, scene, barycenters, candidates);
			
	return extrinsics;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<class CameraModel, class Observations>
void init_extrinsic(
	//OUT
	CalibrationPose& extrinsic,
	//IN
	const CameraModel& model,
	const PointsConstellation & scene,
	const Observations& observations,
	//GUI
	const Image& picture/* for GUI only */
)
{
	PRINT_WARN("Ensure that features are linked to corresponding cluster.");
	
	constexpr int f = -1;
	const bool usePicture = (picture.cols > 0u);
	
	//Configure monocular camera
	Sensor film = Sensor{model.sensor().width(), model.sensor().height(), model.sensor().scale()};
	film.pose() = model.sensor().pose();
#if defined(INIT_AT_FOCAL_LENGTH) && INIT_AT_FOCAL_LENGTH
	film.pose().translation().z() = - model.focal();
#endif
	
	const PinholeCamera monocular{model.focal(), film};
	
	//Estimate barycenters
	PRINT_DEBUG("Estimate barycenters of frame");
	Observations barycenters = compute_barycenters(observations); //IMAGE UV
		
	GUI(
		if (usePicture)
		{	
			RENDER_DEBUG_2D(
				Viewer::context().layer(Viewer::layer()).name("Frame"), 
				picture
			);	
			Viewer::update();
		}
		PRINT_DEBUG("[GUI] Display information of frame");
		display(f, observations);
		display(f, barycenters, tag::Barycenters{});	
	);
				
	//Estimate Pose
	PRINT_DEBUG("Estimate Pose of frame");
	const auto [pose, rmse] = estimate_pose(model, scene, barycenters);
		
	PRINT_DEBUG("Sanity check of pose frame f = " << f);
	if( pose.translation()[2] > 0. 
		or rmse.get() > 1e12
		or not(((pose.translation().array() == pose.translation().array())).all()) //check is_nan
		or not(((pose.rotation().array() == pose.rotation().array())).all()) //check is_nan
	) 
	{
		PRINT_ERR("Wrong hypothesis. Can't fix it. Remove pose and observations of frame f = " << f <<".");
		DEBUG_VAR(pose);
		DEBUG_VAR(rmse.get());
		
		wait();
		return;
	}
	
	DEBUG_VAR(pose);
	
	GUI(	
		Observations ubarycenters = compute_barycenters(observations);			
		display(f, observations);
		display(f, ubarycenters, tag::Barycenters{});
		wait();	
	);
		
	extrinsic = CalibrationPose{pose, -1};
}

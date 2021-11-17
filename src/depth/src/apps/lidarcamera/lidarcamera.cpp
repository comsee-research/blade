//STD
#include <iostream>
#include <unistd.h>

#include <opencv2/photo.hpp>

//LIBPLENO
#include <pleno/types.h>

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

//geometry
#include <pleno/geometry/observation.h>
#include <pleno/geometry/depth/pointcloud.h>

//processing
#include <pleno/processing/detection/detection.h> 
#include <pleno/processing/imgproc/improcess.h> //devignetting, erode, dilate

#include "../../processing/calibration/calibration.h" 
#include "../../processing/depth/depth.h" 
#include "../../processing/depth/initialization.h" 
#include "../../processing/depth/filter.h" 

#include "../../processing/tools/chrono.h"

//graphic
#include "../../graphic/display.h" 

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/observations.h>
#include <pleno/io/cfg/poses.h>

#include "../../io/cfg/depths.h"

#include <pleno/io/images.h>
#include "io/depths.h"

#include "utils.h"

int main(int argc, char* argv[])
{
	PRINT_INFO("========= Lidar-Camera Calibration with a Multifocus plenoptic camera =========");
	Config_t config = parse_args(argc, argv);
	
	Viewer::enable(config.use_gui); DEBUG_VAR(Viewer::enable());
	
	Printer::verbose(config.verbose); DEBUG_VAR(Printer::verbose());
	Printer::level(config.level); DEBUG_VAR(Printer::level());

////////////////////////////////////////////////////////////////////////////////
// 1) Load Image from configuration file
////////////////////////////////////////////////////////////////////////////////
	ImageWithInfo image;
	Image mask;
	std::size_t imgformat = 8;
	{
		PRINT_WARN("1) Load Images from configuration file");
		ImagesConfig cfg_images;
		v::load(config.path.images, cfg_images);
		DEBUG_ASSERT((cfg_images.meta().rgb()), "Images must be in rgb format.");
		DEBUG_ASSERT((cfg_images.meta().format() < 16), "Floating-point images not supported.");
		
		imgformat = cfg_images.meta().format();
	
		//1.2) Load checkerboard images
		PRINT_WARN("\t1.1) Load image");	
		load(cfg_images.images()[0], image, cfg_images.meta().debayered());
		const double cbfnbr = image.fnumber;	
		
		//1.3) Load white image corresponding to the aperture (mask)
		PRINT_WARN("\t1.2) Load white image corresponding to the aperture (mask)");
		ImageWithInfo mask_;
		load(cfg_images.mask(), mask_, cfg_images.meta().debayered());
		
		const auto [mimg, mfnbr, __] = mask_;
		mask = mimg;
		DEBUG_ASSERT((mfnbr == image.fnumber), "No corresponding f-number between mask and images");
	}
	
	PRINT_WARN("\t1.3) Devignetting images");
	Image picture = [&]() -> Image {
		Image unvignetted;
			
		if (imgformat == 8u) devignetting(image.img, mask, unvignetted);
		else /* if (imgformat == 16u) */ devignetting_u16(image.img, mask, unvignetted);
				
		return unvignetted;	
	}();
	
	Image gray = Image::zeros(picture.rows, picture.cols, CV_8UC1);
	cv::cvtColor(picture, gray, cv::COLOR_BGR2GRAY);

////////////////////////////////////////////////////////////////////////////////
// 2) Load Camera information configuration file
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("2) Load Camera information from configuration file");
	PlenopticCamera mfpc;
	load(config.path.camera, mfpc);
	
	InternalParameters params;
	v::load(config.path.params, v::make_serializable(&params));
	mfpc.params() = params;

	PRINT_INFO("Camera = " << mfpc << std::endl);
	PRINT_INFO("Internal Parameters = " << params << std::endl);

////////////////////////////////////////////////////////////////////////////////
// 3) Load scene information
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("3) Load Scene Model");
	SceneConfig cfg_scene;
	v::load(config.path.scene, cfg_scene);
	DEBUG_ASSERT(
		(cfg_scene.constellations().size() > 0u),
		"No constellation available while loading scene"
	);
	
	PointsConstellation scene{cfg_scene.constellations()[0]};
	for (const auto& p : scene.constellation) DEBUG_VAR(p.transpose());

////////////////////////////////////////////////////////////////////////////////
// 4) Load bap features
////////////////////////////////////////////////////////////////////////////////
	BAPObservations bap_obs;	
	if (config.path.features == "")
	{
		PRINT_WARN("4) Detect Features");
		
		//for each point in the constellation
		for (std::size_t i = 0; i < scene.size(); ++i)
		{
			PRINT_INFO("=== Detecting BAP Observation in image for point (" << i << ") in constellation");
			BAPObservations bapf = detection_bapfeatures(picture, mfpc.mia(), mfpc.params());
			DEBUG_VAR(bapf.size());
			
			//assign frame index
			std::for_each(bapf.begin(), bapf.end(), [&i](BAPObservation& cbo) { cbo.frame = 0; cbo.cluster = i; });
				
			//update observations				
			bap_obs.insert(std::end(bap_obs), 
				std::make_move_iterator(std::begin(bapf)),
				std::make_move_iterator(std::end(bapf))
			);	
			DEBUG_VAR(bap_obs.size());
		}
		
		bap_obs.shrink_to_fit();
		
		ObservationsConfig cfg_obs;
		cfg_obs.features() = bap_obs;
		cfg_obs.centers() = MICObservations{};
		
		v::save("observations-"+std::to_string(getpid())+".bin.gz", cfg_obs);
		
		DEBUG_VAR(cfg_obs.features().size());
	}
	else
	{
		PRINT_WARN("4) Load Features");
		ObservationsConfig cfg_obs;
		v::load(config.path.features, cfg_obs);
		DEBUG_VAR(cfg_obs.features().size());
		bap_obs = std::move(cfg_obs.features());
		DEBUG_VAR(bap_obs.size());
	}
	
////////////////////////////////////////////////////////////////////////////////
// 5) Optimize
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("5) Calibration lidar-camera");	
	PRINT_WARN("\t5.1) Load initial pose");
	CalibrationPoseConfig cfg_pose;
	v::load(config.path.extrinsics, cfg_pose);
	
	CalibrationPose pose{cfg_pose.pose(), cfg_pose.frame()};
	DEBUG_VAR(pose.pose);

	PRINT_WARN("\t5.2) Run calibration");
	calibration_LidarPlenopticCamera(pose, mfpc, scene, bap_obs, picture);
	
////////////////////////////////////////////////////////////////////////////////
// 6) PointCloud computation
////////////////////////////////////////////////////////////////////////////////s
	if (config.path.pc == "")
	{
		PRINT_WARN("6) Computing PointCloud");
		DepthMap dm;
		
		if (config.path.dm == "")
		{
			PRINT_WARN("\t6.1) Load depth estimation config");		
			DepthEstimationStrategy strategies;
			v::load(config.path.strategy, v::make_serializable(&strategies));
			
			PRINT_WARN("\t6.2) Estimate depthmaps");	
			const auto [mind, maxd] = initialize_min_max_distance(mfpc);
			const double dmin = strategies.dtype == DepthMap::DepthType::VIRTUAL ? 
					strategies.vmin /* mfpc.obj2v(maxd) */
				: 	std::max(mfpc.v2obj(strategies.vmax), mind);
			
			const double dmax = strategies.dtype == DepthMap::DepthType::VIRTUAL ? 
					strategies.vmax /* mfpc.obj2v(mind) */
				: 	std::min(mfpc.v2obj(strategies.vmin), maxd);
				
			const std::size_t W = strategies.mtype == DepthMap::MapType::COARSE ? 
					mfpc.mia().width() 
				: 	mfpc.sensor().width();
				
			const std::size_t H = strategies.mtype == DepthMap::MapType::COARSE ? 
					mfpc.mia().height() 
				: 	mfpc.sensor().height();
				
			DepthMap tdm{
				W, H, dmin, dmax,
				strategies.dtype, strategies.mtype
			};

			estimate_depth(tdm, mfpc, gray, strategies, picture, false);
			v::save("dm-"+std::to_string(getpid())+".bin.gz", v::make_serializable(&tdm));
			clear();
			
			config.path.dm = "./dm-"+std::to_string(getpid())+".bin.gz";
		}
		
		v::load(config.path.dm, v::make_serializable(&dm));
		inplace_minmax_filter_depth(dm, mfpc.obj2v(1500.), mfpc.obj2v(400.));
		
		//FIXME: filter should be applied on metric dm, as all virtual depth hypotheses are in the same unit

		PointCloud pc = [&]() -> PointCloud {
			DepthMap mdm = dm.to_metric(mfpc);
			
			//if (mdm.is_refined_map()) inplace_median_filter_depth(mdm, mfpc, AUTOMATIC_FILTER_SIZE, true);
			//inplace_median_filter_depth(mdm, mfpc, AUTOMATIC_FILTER_SIZE, false);		
			
			//if (mdm.is_refined_map()) inplace_bilateral_filter_depth(mdm, mfpc, 10., 1., true);
			//inplace_bilateral_filter_depth(mdm, mfpc, 10., AUTOMATIC_FILTER_SIZE, false);	
			
			//inplace_consistency_filter_depth(mdm, mfpc, 10. /* mm */);	
				
			inplace_minmax_filter_depth(mdm, 400., 1500.);
			display(mdm, mfpc);
		
			return PointCloud{mdm, mfpc, picture};
		}();		
		v::save("pc-"+std::to_string(getpid())+".bin.gz", v::make_serializable(&pc));
		
		config.path.pc = std::string("pc-"+std::to_string(getpid())+".bin.gz");
		wait();
	}
	else
	{
		PRINT_WARN("6) Load pointcloud");
	}
	
FORCE_GUI(true);	
	constexpr std::size_t maxcount = 500'000;
	
	PointCloud pc; 
	v::load(config.path.pc, v::make_serializable(&pc));
	inplace_minmax_filter_depth(pc, 400., 1500., Axis::Z);
	inplace_maxcount_filter_depth(pc, maxcount);
	DEBUG_VAR(pc.size());
	
	PRINT_WARN("7) Graphically checking point cloud transform");
	const CalibrationPose porigin{Pose{}, -1}; 
	display(porigin); display(scene); display(pose);
	
	display(1, pc);
	
	PointsConstellation initial_constellation, final_constellation;
	for	(const P3D& pc : scene)
	{
		const P3D p = to_coordinate_system_of(cfg_pose.pose(), pc);
		initial_constellation.add(p);
		
		const P3D q = to_coordinate_system_of(pose.pose, pc);
		final_constellation.add(q);
	}
	display(initial_constellation, 10.); //constellation transformed, coord in (0,0,0), i.e. camera frame
	display(final_constellation); 
	
	wait();
	
	if (config.path.pts != "")
	{
		PointCloud reference = read_pts(config.path.pts);		
		display(2, reference);	
		
		PointCloud transformed_pc = reference;
		transformed_pc.transform(pose.pose);
		inplace_minmax_filter_depth(transformed_pc, 400., 1500., Axis::Z);		
		//inplace_maxcount_filter_depth(transformed_pc, maxcount);
		DEBUG_VAR(transformed_pc.size());
		
		display(3, transformed_pc);			
		
		wait();	
	}
FORCE_GUI(false);		

	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}


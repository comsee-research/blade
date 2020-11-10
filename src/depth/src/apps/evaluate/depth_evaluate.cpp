//STD
#include <iostream>
#include <unistd.h>
#include <variant> //std::variant
//EIGEN
//BOOST
//OPENCV
#include <opencv2/opencv.hpp>

//LIBPLENO
#include <pleno/types.h>

#include <pleno/graphic/gui.h>
#include "../../graphic/display.h"

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

//geometry
#include <pleno/geometry/observation.h>
#include "geometry/depth/RawCoarseDepthMap.h"

//processing
#include <pleno/processing/improcess.h> //devignetting
#include "processing/depth/initialization.h"
#include "processing/depth/export.h"

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/observations.h>
#include <pleno/io/cfg/poses.h>

#include "utils.h"

#include "load.h"
#include "reduce.h"
#include "compute.h"
#include "export.h"

int main(int argc, char* argv[])
{
	PRINT_INFO("========= Depth Evaluation with a Multifocus plenoptic camera =========");
	Config_t config = parse_args(argc, argv);
	
	Viewer::enable(config.use_gui); DEBUG_VAR(Viewer::enable());
	
	Printer::verbose(config.verbose); DEBUG_VAR(Printer::verbose());
	Printer::level(config.level); DEBUG_VAR(Printer::level());

////////////////////////////////////////////////////////////////////////////////
// 1) Load Camera information configuration file
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("1) Load Camera information from configuration file");
	PlenopticCamera mfpc;
	load(config.path.camera, mfpc);
	
	InternalParameters params;
	v::load(config.path.params, v::make_serializable(&params));
	mfpc.params() = params;

	PRINT_INFO("Camera = " << mfpc << std::endl);
	PRINT_INFO("Internal Parameters = " << params << std::endl);

////////////////////////////////////////////////////////////////////////////////
// 2) Evaluate depths
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("2) Starting depth evaluation");
	std::unordered_map<Index /* frame index */, BAPObservations> obs;
	
	if (config.path.features != "")
	{
		PRINT_WARN("\t2.0) Load Features");	

		ObservationsConfig cfg_obs;
		v::load(config.path.features, cfg_obs);

		BAPObservations observations = cfg_obs.features(); 
		DEBUG_VAR(observations.size());
		
		DEBUG_ASSERT(
			((observations.size() > 0u)), 
			"No observations available (missing features)"
		);
		
		//split observations according to frame index
		for (const auto& ob : observations)
			obs[ob.frame].push_back(ob);	
	}	
	
	PRINT_WARN("\t2.1) Load ground truth displacements");	
	//load ground truth
	std::map<Index, double> gtdepth = load_gt_dist(config.path.gt);
	
	using DepthMapsType = std::variant<
		std::map<Index, RawCoarseDepthMap>,
		std::map<Index, XYZs>,
		std::map<Index, Pose>
	>;
	
	PRINT_WARN("\t2.2) Load depth maps");	
	DepthMapsType vdms;
	
	if (config.path.dm != "") //evaluate depth from depthmap
	{
		DepthMapsConfig cfg;
		v::load(config.path.dm, cfg);	
		
		// load
		vdms.emplace<std::map<Index, RawCoarseDepthMap>>(load(cfg, mfpc));	
	}
	else if (config.path.xyz != "")
	{
		XYZsConfig cfg;
		v::load(config.path.xyz, cfg);
	
		// load
		vdms.emplace<std::map<Index, XYZs>>(load(cfg));
	}
	else if (config.path.poses != "")
	{
		CalibrationPosesConfig cfg;
		v::load(config.path.poses, cfg);
		
		// load
		vdms.emplace<std::map<Index, Pose>>(load(cfg));
	}
	else
	{
		PRINT_ERR("No depthmaps representation has been given. Abort.");
		abort();
	}
	
	std::visit([&](auto&& dms){
		using T = std::decay_t<decltype(dms)>;
		
		PRINT_WARN("\t2.3) Estimate mean depth for each depth map");	
		//reduce
		std::map<Index, double> depths;
		
		if constexpr (std::is_same_v<T, std::map<Index, RawCoarseDepthMap>>)
		{
			if (config.path.features != "") depths = reduce(dms, obs);
			else depths = reduce(dms);
		}
		else depths = reduce(dms);
		
		//compute
		PRINT_WARN("\t2.4) Compute depth errors");	
		DepthError err = compute(depths, gtdepth);
		
		//save
		PRINT_WARN("\t2.5) Save errors");	
		export_depth_errors(err);

	}, vdms);

	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}


//STD
#include <iostream>
#include <unistd.h>

//LIBPLENO
#include <pleno/types.h>

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

//geometry
#include <pleno/geometry/observation.h>
#include "geometry/depth/RawDepthMap.h"

//processing
#include <pleno/processing/calibration/init.h> 
#include <pleno/processing/imgproc/improcess.h> //devignetting
#include <pleno/processing/tools/stats.h> //median

#include "../../processing/calibration/calibration.h" 

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/observations.h>
#include <pleno/io/cfg/poses.h>
#include "io/cfg/depthmaps.h"

#include <pleno/io/images.h>

#include "eval.h"
#include "utils.h"

int main(int argc, char* argv[])
{
	PRINT_INFO("========= Depth Calibration with a Multifocus plenoptic camera =========");
	Config_t config = parse_args(argc, argv);
	
	Viewer::enable(config.use_gui); DEBUG_VAR(Viewer::enable());
	
	Printer::verbose(config.verbose); DEBUG_VAR(Printer::verbose());
	Printer::level(config.level); DEBUG_VAR(Printer::level());

////////////////////////////////////////////////////////////////////////////////
// 1) Load Images from configuration file
////////////////////////////////////////////////////////////////////////////////
	std::vector<ImageWithInfo> images;
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
		PRINT_WARN("\t1.1) Load images");	
		load(cfg_images.images(), images, cfg_images.meta().debayered());
		
		DEBUG_ASSERT((images.size() != 0u),	"You need to provide images!");
		
		const double cbfnbr = images[0].fnumber;	
		for (const auto& [ _ , fnumber, __] : images)
		{
			DEBUG_ASSERT((cbfnbr == fnumber), "All images should have the same aperture configuration");
		}
		
		//1.3) Load white image corresponding to the aperture (mask)
		PRINT_WARN("\t1.2) Load white image corresponding to the aperture (mask)");
		ImageWithInfo mask_;
		load(cfg_images.mask(), mask_, cfg_images.meta().debayered());
		
		const auto [mimg, mfnbr, __] = mask_;
		mask = mimg;
		DEBUG_ASSERT((mfnbr == cbfnbr), "No corresponding f-number between mask and images");
	}
	
	PRINT_WARN("\t1.3) Devignetting images");
	std::unordered_map<Index, Image> pictures;
	for (const auto& iwi : images)
	{
		Image unvignetted;
			
		if (imgformat == 8u) devignetting(iwi.img, mask, unvignetted);
		else /* if (imgformat == 16u) */ devignetting_u16(iwi.img, mask, unvignetted);
		
		Image img = Image::zeros(unvignetted.rows, unvignetted.cols, CV_8UC1);
		cv::cvtColor(unvignetted, img, cv::COLOR_BGR2GRAY);
		
		pictures[iwi.frame] = std::move(img);	
	}
	
	DEBUG_VAR(pictures.size());

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
		(cfg_scene.checkerboards().size() > 0u),
		"No model available while loading scene"
	);
	
	CheckerBoard scene{cfg_scene.checkerboards()[0]};

////////////////////////////////////////////////////////////////////////////////
// 4) Load bap features
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("4) Load Features");
	ObservationsConfig cfg_obs;
	v::load(config.path.features, cfg_obs);
		
	DEBUG_ASSERT(
		(cfg_obs.features().size() > 0u), 
		"No observations available (missing features)"
	);
	
	CalibrationPoses poses;
		
	BAPObservations bap_obs;
	bap_obs.reserve(cfg_obs.features().size());

	PRINT_INFO("4.1) Link observations");
	init_extrinsics(
		bap_obs, poses,
		mfpc, scene, cfg_obs.features(), {}
	);
	
	PRINT_INFO("4.2) Split observations");
	std::unordered_map<Index, BAPObservations> observations;
	for(const auto& ob : bap_obs) observations[ob.frame].push_back(ob);

	DEBUG_VAR(observations.size());
	
////////////////////////////////////////////////////////////////////////////////
// 5) Load depth maps
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("5) Load depth maps");	
	DepthMapsConfig cfg_dms;
	v::load(config.path.dm, cfg_dms);
	
	std::unordered_map<Index, RawDepthMap> depthmaps;
	
	for (auto & cfg_dm : cfg_dms.maps())
	{	
		RawDepthMap dm{mfpc};
		v::load(cfg_dm.path(), v::make_serializable(&dm));
		
		depthmaps.emplace(cfg_dm.frame(), std::move(dm));
	}
	
	DEBUG_VAR(depthmaps.size());
#if 0
	DEBUG_ASSERT(
		(pictures.size() == observations.size() and observations.size() == depthmaps.size()), 
		"No matching between depthmap, images and features"
	);
#endif

////////////////////////////////////////////////////////////////////////////////
// 6) Start processing
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("6) Calibrate scale error");
	
	PRINT_INFO("\t6.1) Evaluate initial scale error");
	evaluate_scale_error(mfpc, scene, depthmaps, observations, pictures);
	
	PRINT_INFO("\t6.2) Calibrate scale");
	LinearFunction scaling;	
	calibration_depthScaling(scaling, mfpc, scene, depthmaps, observations);
	
	PRINT_INFO("\t6.3) Evaluate rescaled error");
	evaluate_scale_error(mfpc, scaling, scene, depthmaps, observations, pictures);
	
	PRINT_INFO("\t6.4) Computing new depth map");
	auto reduce = [](const RawDepthMap& dm) -> double {
		std::vector<double> zs; zs.reserve(dm.width() * dm.height());		
		for (std::size_t k = 0; k < dm.width(); ++k)
			for (std::size_t l = 0; l < dm.height(); ++l)
				if (dm.depth(k,l) != DepthInfo::NO_DEPTH)
					zs.emplace_back(dm.depth(k,l));
					
		zs.shrink_to_fit();
		
		return median(zs);
	};
	
	auto reduce_scaled = [&scaling](const RawDepthMap& dm) -> double {
		std::vector<double> zs; zs.reserve(dm.width() * dm.height());		
		for (std::size_t k = 0; k < dm.width(); ++k)
			for (std::size_t l = 0; l < dm.height(); ++l)
				if (dm.depth(k,l) != DepthInfo::NO_DEPTH)
					zs.emplace_back(scaling(dm.depth(k,l)));
					
		zs.shrink_to_fit();
		
		return median(zs);
	};
	
	for (const auto& [frame, dm] : depthmaps)
	{
		const RawDepthMap mdm = dm.to_metric(mfpc);
		const double z = reduce(mdm);
		const double sz = reduce_scaled(mdm);
		
		PRINT_INFO("Estimated depth of frame ("<< frame <<"): z = " << z << " (mm), rescaled z = " << sz << " (mm)");	
	}
	
	if(save()) 
	{
		mfpc.scaling() = scaling;
		PRINT_WARN("\t... Saving Intrinsic Parameters");
		save("intrinsics-"+std::to_string(getpid())+".js", mfpc);
		
		scaling.a = 1.; scaling.b = 0.;
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}


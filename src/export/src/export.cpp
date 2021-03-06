//STD
#include <iostream>
#include <unistd.h>
//EIGEN
//BOOST
//OPENCV
#include <opencv2/opencv.hpp>

//LIBPLENO
#include <pleno/types.h>

#include <pleno/graphic/gui.h>
#include <pleno/graphic/display.h>

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

//geometry
#include <pleno/geometry/observation.h>
#include <pleno/geometry/depth/depthmap.h>

//processing
#include <pleno/processing/estimation.h> //estimation_plane_fitting/ransac
#include <pleno/processing/imgproc/improcess.h> //devignetting
#include <pleno/processing/depth/initialization.h>

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/observations.h>
#include <pleno/io/cfg/poses.h>

#include <pleno/io/images.h>

#include "helpers.h"
#include "utils.h"

int main(int argc, char* argv[])
{
	PRINT_INFO("========= BLADE export with a Multifocus plenoptic camera =========");
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
// 3) Starting Blur Aware depth estimation
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("3) Starting Blur Aware depth estimation");
	PRINT_WARN("\t3.1) Devignetting images");
			
	std::vector<Image> pictures;
	pictures.reserve(images.size());
	
	std::transform(
		images.begin(), images.end(),
		std::back_inserter(pictures),
		[&mask, &imgformat](const auto& iwi) -> Image { 
			Image unvignetted;
			
			if (imgformat == 8u) devignetting(iwi.img, mask, unvignetted);
			else /* if (imgformat == 16u) */ devignetting_u16(iwi.img, mask, unvignetted);
			
    		Image img = Image::zeros(unvignetted.rows, unvignetted.cols, CV_8UC1);
			cv::cvtColor(unvignetted, img, cv::COLOR_BGR2GRAY);
			return img; 
		}	
	);	
	
	//--------------------------------------------------------------------------
	const auto [mind, maxd] = initialize_min_max_distance(mfpc);	
	if (config.path.dm != "")
	{
		PRINT_WARN("\t3.2) Export depth histogram");
		DepthMap dm;
		v::load(config.path.dm, v::make_serializable(&dm));
		
		DepthMap odm = dm.is_virtual_depth() ? dm.to_metric(mfpc) : dm.to_virtual(mfpc);
		
		PRINT_INFO("=== Exporting histogram");
		export_depth_histogram(dm);	
		export_depth_histogram(odm);
			
		PRINT_INFO("=== Displaying depthmap");
		display(dm, mfpc);
		display(odm, mfpc);		
	}
	else if (config.path.pc != "")
	{
		PRINT_WARN("\t3.2) Export plane from point cloud");
		PointCloud pc;
		v::load(config.path.pc, v::make_serializable(&pc));
		
		PRINT_INFO("=== Estimating plane from pointcloud");
		const Plane plane_fitted = estimation_plane_fitting(pc.features());
		DEBUG_VAR(plane_fitted);
		DEBUG_VAR(plane_fitted.dist());
		DEBUG_VAR(plane_fitted.origin().transpose());
		DEBUG_VAR(plane_fitted.position().transpose());
		
		const std::size_t nbpts = pc.features().size(); 
		DEBUG_ASSERT((nbpts >= 3), "You need at least 3 points to estimate a plane...");
		
		const std::size_t n = std::max(3ul, static_cast<std::size_t>(nbpts * 0.1)); //at least 10% of the points
		const std::size_t d = std::max(n, static_cast<std::size_t>(nbpts * 0.2)); //at least 20% of the points
		constexpr double threshold = 1.; //mm
		
		PRINT_INFO("RANSAC: pointcloud = " << nbpts << " points, n = " << n << ", d = " << d << ", and threshold = " << threshold << " mm");
				
		const Plane plane_ransac = estimation_plane_ransac(pc.features(), threshold, n, 100 /* k */, d);
		DEBUG_VAR(plane_ransac);
		DEBUG_VAR(plane_ransac.dist());
		DEBUG_VAR(plane_ransac.origin().transpose());
		DEBUG_VAR(plane_ransac.position().transpose());		
		
		PRINT_INFO("=== Displaying pointcloud");
		display(mfpc);
		display(0, pc);
	
		display(0, plane_fitted);
		display(1, plane_ransac);
		
		v::save("plane-"+std::to_string(getpid())+".js", v::make_serializable(&plane_ransac));
		
		wait();
	}
	else if (config.path.features != "")
	{
		PRINT_WARN("\t3.2) Load Features");	
		BAPObservations observations;
		{
			ObservationsConfig cfg_obs;
			v::load(config.path.features, cfg_obs);

			observations = cfg_obs.features(); DEBUG_VAR(observations.size());
			
			DEBUG_ASSERT(
				((observations.size() > 0u)), 
				"No observations available (missing features)"
			);
		}	
		
		//split observations according to frame index
		std::unordered_map<Index /* frame index */, BAPObservations> obs;
		for (const auto& ob : observations)
			obs[ob.frame].push_back(ob);	
		
		//for each frame
		for (const auto & [frame, baps]: obs)
		{ 	
			PRINT_INFO("=== Export costfunction at frame = " << frame);	
				
			export_cost_function_from_obs(
				mfpc, pictures[frame], baps, 
				mfpc.obj2v(maxd), mfpc.obj2v(mind), 1000
			);
		}
	}
	else
	{
		PRINT_WARN("\t3.2.1) Compute micro-images blade");	
		do {
			export_micro_images_blade(mfpc, pictures[4]);
		} while (not finished());
		
		PRINT_WARN("\t3.2.2) Compute depth cost function");	
		do {
			export_cost_function(
				mfpc, pictures[0], mfpc.obj2v(maxd), mfpc.obj2v(mind), 2500, 
				ObservationsPairingStrategy::CENTRALIZED, false /* per_baseline */
			);
		} while (not finished());
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}


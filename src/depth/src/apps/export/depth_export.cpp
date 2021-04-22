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
#include "../../graphic/display.h"

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

//geometry
#include <pleno/geometry/observation.h>
#include "geometry/depth/RawDepthMap.h"

//processing
#include <pleno/processing/imgproc/improcess.h> //devignetting
#include "processing/depth/initialization.h"
#include "processing/depth/export.h"

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/observations.h>
#include <pleno/io/cfg/poses.h>

#include <pleno/io/images.h>

#include "utils.h"

int main(int argc, char* argv[])
{
	PRINT_INFO("========= Depth Estimation with a Multifocus plenoptic camera =========");
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
		RawDepthMap dm{mfpc};
		v::load(config.path.dm, v::make_serializable(&dm));
		
		RawDepthMap odm = dm.is_virtual_depth() ? dm.to_metric(mfpc) : dm.to_virtual(mfpc);
		
		PRINT_INFO("=== Exporting histogram");
		export_depth_histogram(dm);	
		export_depth_histogram(odm);
			
		PRINT_INFO("=== Displaying depthmap");
		display(dm);
		display(odm);
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
		PRINT_WARN("\t3.2) Compute depth cost function");	
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


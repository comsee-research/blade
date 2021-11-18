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

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

//geometry
#include <pleno/geometry/observation.h>
#include "geometry/depth/depthmap.h"

//processing
#include <pleno/processing/imgproc/improcess.h> //devignetting
#include "processing/depth/depth.h"
#include "processing/depth/initialization.h"

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
// 3) Features extraction step
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("3) Load Features");	
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

////////////////////////////////////////////////////////////////////////////////
// 4) Starting Blur Aware depth estimation
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("4) Starting Blur Aware depth estimation");
	PRINT_WARN("\t4.1) Devignetting images");
			
	IndexedImages pictures;
	IndexedImages cpictures;
		
	std::transform(
		images.begin(), images.end(),
		std::inserter(pictures, pictures.end()),
		[&mask, &imgformat, &cpictures](const auto& iwi) -> auto { 
			Image unvignetted;
			
			if (imgformat == 8u) devignetting(iwi.img, mask, unvignetted);
			else /* if (imgformat == 16u) */ devignetting_u16(iwi.img, mask, unvignetted);
			
    		Image img = Image::zeros(unvignetted.rows, unvignetted.cols, CV_8UC1);
			cv::cvtColor(unvignetted, img, cv::COLOR_BGR2GRAY);
			
			cpictures[iwi.frame] = std::move(unvignetted); //save color images
			return std::make_pair(iwi.frame, img); 
		}	
	);	
	
	PRINT_WARN("\t4.2) Estimate depth from observations");	
	//split observations according to frame index
	std::unordered_map<Index /* frame index */, BAPObservations> obs;
	for (const auto& ob : observations)
		obs[ob.frame].push_back(ob);	
	
	//for each frame
	int cindex = 0;
	for (auto & [frame, baps]: obs)
	{ 	
		try { pictures.at(frame); }
		catch (std::out_of_range&) { continue; }
		
		PRINT_INFO("=== Estimate depth from observations at frame = " << frame);	
		const auto [mind, maxd] = initialize_min_max_distance(mfpc);
		DepthMap dm{mfpc.mia().width(), mfpc.mia().height(), mfpc.obj2v(maxd), mfpc.obj2v(mind)};
		
		estimate_depth_from_obs(
			dm, mfpc, pictures[frame], baps
		);
		
		if (save())
		{
			PRINT_INFO("=== Saving depthmap... ");
			std::ostringstream name; 
			name << "odm-";
			if (mfpc.I() > 0u) name << "blade-";
			else name << "disp-";
			
			name << frame << "-" << getpid() << ".bin.gz";
			
			v::save(name.str(), v::make_serializable(&dm));
		}
		
		if (finished()) break;
		clear();
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}


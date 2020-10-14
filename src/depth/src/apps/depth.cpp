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
#include "io/choice.h"

//geometry
#include <pleno/geometry/observation.h>

//processing
#include <pleno/processing/improcess.h> //devignetting
#include "processing/depth/depth.h"
#include "processing/depth/strategy.h"
#include "processing/depth/initialization.h"

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/observations.h>
#include <pleno/io/cfg/poses.h>

#include "utils.h"

void load(const std::vector<ImageWithInfoConfig>& cfgs, std::vector<ImageWithInfo>& images)
{
	images.reserve(cfgs.size());
	
	for(const auto& cfg : cfgs)
	{
		PRINT_DEBUG("Load image " << cfg.path());
		images.emplace_back(
			ImageWithInfo{ 
				cv::imread(cfg.path(), cv::IMREAD_UNCHANGED),
				cfg.fnumber()
			}
		);	
	}
}

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
	{
		PRINT_WARN("1) Load Images from configuration file");
		ImagesConfig cfg_images;
		v::load(config.path.images, cfg_images);
	
		//1.2) Load checkerboard images
		PRINT_WARN("\t1.1) Load images");	
		load(cfg_images.images(), images);
		
		DEBUG_ASSERT((images.size() != 0u),	"You need to provide images!");
		
		const double cbfnbr = images[0].fnumber;	
		for (const auto& [ _ , fnumber] : images)
		{
			DEBUG_ASSERT((cbfnbr == fnumber), "All images should have the same aperture configuration");
		}
		
		//1.3) Load white image corresponding to the aperture (mask)
		PRINT_WARN("\t1.2) Load white image corresponding to the aperture (mask)");
		const auto [mask_, mfnbr] = ImageWithInfo{ 
					cv::imread(cfg_images.mask().path(), cv::IMREAD_UNCHANGED),
					cfg_images.mask().fnumber()
				};
		mask = mask_;
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
		[&mask](const auto& iwi) -> Image { 
			Image unvignetted;
			devignetting(iwi.img, mask, unvignetted);
    		Image img = Image::zeros(unvignetted.rows, unvignetted.cols, CV_8UC1);
			cv::cvtColor(unvignetted, img, cv::COLOR_BGR2GRAY);
			return img; 
		}	
	);	
	
	
	PRINT_WARN("\t3.2) Load depth estimation config");
	SearchStrategy search = SearchStrategy(config.method);
		
	DepthEstimationStrategy strategies{
		InitStrategy::REGULAR_GRID, 
		ObservationsPairingStrategy::CENTRALIZED,
		BeliefPropagationStrategy::NONE, 
		search
	};
	PRINT_INFO(strategies);
	
	PRINT_WARN("\t3.3) Estimate depthmaps");	
	const auto [mind, maxd] = initialize_min_max_distance(mfpc);
	
	for (std::size_t frame = 0; frame < pictures.size(); ++frame)
	{
		PRINT_INFO("=== Estimate depth of frame = " << frame);	
		RawCoarseDepthMap dm{mfpc, mfpc.obj2v(maxd), mfpc.obj2v(mind)};
	
		if (config.use_probabilistic)
		{	
			RawCoarseDepthMap confidencedm{mfpc, 0., 10.};
			estimate_probabilistic_depth(dm, confidencedm, mfpc, pictures[frame], strategies);
		}
		else
		{
			estimate_depth(dm, mfpc, pictures[frame], strategies);
		}
		
		if (save())
		{
			PRINT_INFO("=== Saving depthmap...");
			std::ostringstream name; 
			
			if (config.use_probabilistic) name << "pdm-";
			else name << "dm-";
			
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


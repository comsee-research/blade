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
#include <pleno/graphic/viewer_2d.h>
#include <pleno/graphic/viewer_3d.h>

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

//geometry
#include <pleno/geometry/observation.h>
#include "geometry/depth/depthmap.h"
#include "geometry/depth/pointcloud.h"
#include "geometry/depth/depthimage.h"

//processing
#include <pleno/processing/imgproc/improcess.h> //devignetting
#include "processing/depth/depth.h"
#include "processing/depth/strategy.h"
#include "processing/depth/initialization.h"
#include "processing/depth/filter.h"

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
	
	PRINT_WARN("\t3.2) Load depth estimation config");		
	DepthEstimationStrategy strategies;
	v::load(config.path.strategy, v::make_serializable(&strategies));
	
	PRINT_INFO(strategies);
	
	PRINT_WARN("\t3.3) Estimate depthmaps");	
	const auto [mind_, maxd_] = initialize_min_max_distance(mfpc);
	const double dmin = strategies.dtype == DepthMap::DepthType::VIRTUAL ? 
			strategies.vmin /* mfpc.obj2v(maxd_) */
		: 	std::max(mfpc.v2obj(strategies.vmax), mind_);
	
	const double dmax = strategies.dtype == DepthMap::DepthType::VIRTUAL ? 
			strategies.vmax /* mfpc.obj2v(mind) */
		: 	std::min(mfpc.v2obj(strategies.vmin), maxd_);
		
	const std::size_t W = strategies.mtype == DepthMap::MapType::COARSE ? mfpc.mia().width() : mfpc.sensor().width();
	const std::size_t H = strategies.mtype == DepthMap::MapType::COARSE ? mfpc.mia().height() : mfpc.sensor().height();
		
	constexpr double maxd = 1500.;
	constexpr double mind = 400.;
	
	//for (std::size_t frame = 0; frame < pictures.size(); ++frame)
	for (const auto& [frame, picture] : pictures)
	{
		PRINT_INFO("=== Estimate depth of frame = " << frame);	
		DepthMap dm{
			W, H, dmin, dmax,
			strategies.dtype, strategies.mtype
		};
	
		estimate_depth(dm, mfpc, picture, strategies, cpictures[frame]);
		inplace_minmax_filter_depth(dm, mfpc.obj2v(maxd), mfpc.obj2v(mind));
		
		if (config.save_all or save())
		{
			PRINT_INFO("=== Saving depthmap...");
			{
				DepthMapImage dmi = DepthMapImage{dm, mfpc, mfpc.obj2v(maxd), mfpc.obj2v(mind)};
	  			cv::imwrite("dm-"+std::to_string(frame)+"-"+std::to_string(getpid())+".png", dmi.image);
	  			
				std::ostringstream name; 
				
				if (strategies.probabilistic) name << "pdm-";
				else name << "dm-";
				
				if (mfpc.I() > 0u) name << "blade-";
				else name << "disp-";
				
				name << frame << "-" << getpid() << ".bin.gz";
				
				v::save(name.str(), v::make_serializable(&dm));
				
			}			
			
			PRINT_INFO("=== Saving pointcloud...");
			{
				PointCloud pc = PointCloud{dm, mfpc, cpictures[frame]};
				inplace_minmax_filter_depth(pc, mind, maxd, Axis::Z);
				
				std::ostringstream name; 
				
				if (strategies.probabilistic) name << "ppc-";
				else name << "pc-";
				
				if (mfpc.I() > 0u) name << "blade-";
				else name << "disp-";
				
				name << frame << "-" << getpid() << ".bin.gz";
				
				v::save(name.str(), v::make_serializable(&pc));	
				
				PRINT_INFO("=== Saving central view depth map...");
				DepthMapImage dmi = DepthMapImage{pc, mfpc, mind, maxd};
				//cv::cvtColor(dmi.image, dmi.image, CV_BGR2RGB);
				{
					Image cleaned;
					cv::medianBlur(dmi.image, cleaned, 5);
					erode(cleaned, cleaned, 2);
	  				cv::imwrite("csai-"+std::to_string(frame)+"-dm-"+std::to_string(getpid())+".png", cleaned);
  				}
				{
					Image cleaned;
					cv::medianBlur(dmi.depthmap, cleaned, 5);
					erode(cleaned, cleaned, 2);
	  				cv::imwrite("fcsai-"+std::to_string(frame)+"-dm-"+std::to_string(getpid())+".exr", cleaned);
  				}
			}		
		}
		
		if (not(config.run_all) and finished()) break;
		clear();
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}


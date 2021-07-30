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
#include "../../geometry/depth/pointcloud.h"

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

#include "io/cfg/depths.h"

#include <pleno/io/images.h>
#include "io/depths.h"

#include "utils.h"

int main(int argc, char* argv[])
{
	PRINT_INFO("========= PointCloud distances with a Multifocus plenoptic camera =========");
	Config_t config = parse_args(argc, argv);
	
	Viewer::enable(config.use_gui); DEBUG_VAR(Viewer::enable());
	
	Printer::verbose(config.verbose); DEBUG_VAR(Printer::verbose());
	Printer::level(config.level); DEBUG_VAR(Printer::level());

////////////////////////////////////////////////////////////////////////////////
// 1) Load Image from configuration file
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
	IndexedImages pictures;
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
// 5) Optimize
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("3) Load extrinsic pose");
	CalibrationPoseConfig cfg_pose;
	v::load(config.path.extrinsics, cfg_pose);
	
	const Pose extrinsics = cfg_pose.pose();
	DEBUG_VAR(extrinsics);
	
////////////////////////////////////////////////////////////////////////////////
// 4) PointCloud computation
////////////////////////////////////////////////////////////////////////////////
	std::map<Index, PointCloud> pointclouds;
	DepthsConfig depths_cfg;
	
	if (config.path.pc == "")
	{
		PRINT_WARN("4) Compute PointCloud");
		PRINT_WARN("\t4.1) Loading depthmaps");
		v::load(config.path.dm, depths_cfg);
		
		std::map<Index, DepthMap> depthmaps = load(depths_cfg.maps());	
		
		PRINT_WARN("\t4.2) Converting to pointclouds");
		for (auto& [frame, dm] : depthmaps)
		{
			inplace_minmax_filter_depth(dm, mfpc.obj2v(1500.), mfpc.obj2v(400.));
			//FIXME: filter should be applied on metric dm, as all virtual depth hypotheses are in the same unit

			PointCloud pc = [&]() -> PointCloud {
				DepthMap mdm = dm.to_metric(mfpc);
				
				//FIXME: apply filters here
				//if (mdm.is_refined_map()) inplace_median_filter_depth(mdm, mfpc, AUTOMATIC_FILTER_SIZE, true);
				//inplace_median_filter_depth(mdm, mfpc, AUTOMATIC_FILTER_SIZE, false);		
				
				//if (mdm.is_refined_map()) inplace_bilateral_filter_depth(mdm, mfpc, 10., 1., true);
				//inplace_bilateral_filter_depth(mdm, mfpc, 10., AUTOMATIC_FILTER_SIZE, false);	
				
				//inplace_consistency_filter_depth(mdm, mfpc, 10. /* mm */);	
					
				inplace_minmax_filter_depth(mdm, 400., 1500.);		
				display(mdm, mfpc);
			
				return PointCloud{mdm, mfpc, pictures[frame]};
			}();		
			v::save("pc-"+std::to_string(frame)+"-"+std::to_string(getpid())+".bin.gz", v::make_serializable(&pc));
			
			pointclouds.emplace(int(frame), std::move(pc));
						
			if (yes_no_question("Save as .csv"))//save csv
			{
				std::ofstream ofs(
					"pc-"+std::to_string(frame)+"-"+std::to_string(getpid())+".csv"
				); 
				if (not ofs.good()) throw std::runtime_error(std::string("Cannot open file pc.csv"));
				
				std::ostringstream headercsv;
				headercsv << "x,y,z\n";
				
				ofs << headercsv.str();
				
				std::ostringstream oss;
				
				for (const auto& f : pc)
				{
					oss << f.x() << "," << f.y() << "," << f.z() << "\n";			
				}
				
				ofs << oss.str();
				ofs.close();
			}
		}
	}
	else
	{
		PRINT_WARN("4) Load pointclouds");
		v::load(config.path.pc, depths_cfg);
		
		pointclouds = load(depths_cfg.pointclouds());
		DEBUG_VAR(pointclouds.size());	
		for (auto& [frame, pc] : pointclouds)
		{	
			PRINT_INFO("pc (" << frame <<") = " << pc.size());
		}
	}
//FORCE_GUI(true);
////////////////////////////////////////////////////////////////////////////////
// 5) PointCloud processing
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("5) Process pointclouds");
	std::map<Index, PointCloud> rpjpointclouds;
	constexpr std::size_t maxcount = 50'000;
	
	for (auto& [frame, pc] : pointclouds)
	{	
		PRINT_INFO("=== Displaying pc ("<< frame <<")");
		
		inplace_minmax_filter_depth(pc, 400., 1500., Axis::Z);
		display(- 100 - frame, pc);
		
		PRINT_INFO("=== Processing pc ("<< frame <<")");
				
		if (yes_no_question("Do you want to compute CSAI"))
		{
		Chrono::tic();
			DepthMapImage dmi = DepthMapImage{pc, mfpc};
		Chrono::tac();
			cv::cvtColor(dmi.image, dmi.image, CV_BGR2RGB);
			RENDER_DEBUG_2D(
				Viewer::context().layer(Viewer::layer()++)
					.name("CSAI Depth map"),
				dmi.image
		  	);
			PRINT_DEBUG("CSAI depth map (in " << Chrono::get() << " s)");
			
			Image cleaned;
			erode(dmi.image, cleaned, 2);
			dilate(cleaned, cleaned, 2);
			cv::medianBlur(cleaned, cleaned, 5);
			
			#if 0 //inpainting
				Image immask(cleaned.size(),CV_8UC1);
				cv::cvtColor(cleaned, immask, cv::COLOR_RGB2GRAY);
				cv::bitwise_not(immask, immask);
				
				cv::inpaint(cleaned, immask, cleaned, 10, cv::INPAINT_TELEA);	
			#endif
			RENDER_DEBUG_2D(
				Viewer::context().layer(Viewer::layer()++)
					.name("CSAI Depth map (cleaned)"),
				cleaned
		  	);
		  	cv::imwrite("csai-dm-"+std::to_string(frame)+"-"+std::to_string(getpid())+".png", cleaned);
		}
		
		if (yes_no_question("Do you want to reproject pointcloud"))
		{
			inplace_maxcount_filter_depth(pc, maxcount);
			
			DEBUG_VAR(pc.size());
		Chrono::tic();	
			DepthMap reprojecteddm = DepthMap{pc, mfpc}; 
		Chrono::tac();
			PRINT_DEBUG("PointCloud has been reprojected as depth map (in " << Chrono::get() << " s)");
			
			inplace_median_filter_depth(reprojecteddm, mfpc, AUTOMATIC_FILTER_SIZE, true);
			inplace_median_filter_depth(reprojecteddm, mfpc, AUTOMATIC_FILTER_SIZE, false);
						
			v::save("rpjdm-"+std::to_string(frame)+"-"+std::to_string(getpid())+".bin.gz", v::make_serializable(&reprojecteddm));
			
			rpjpointclouds.emplace(int(frame), PointCloud{reprojecteddm, mfpc, pictures[frame]});
			DEBUG_VAR(rpjpointclouds[frame].size());
					
			display(reprojecteddm, mfpc);
			display(-frame, rpjpointclouds[frame]);
		}
		
		wait();	
	}
	
////////////////////////////////////////////////////////////////////////////////
// 6) PointCloud comparison to reference
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("6) Compare pointclouds");	
	PRINT_WARN("\t6.1) Loading reference pointclouds");	
	std::map<Index, PointCloud> references = load(depths_cfg.ptss());
	
	PRINT_WARN("\t6.2) Transforming reference pointclouds");	
	for (auto& [frame, ref] : references)
	{
		const auto pcit = pointclouds.find(int(frame));
		if (pcit == pointclouds.cend()) continue;
		
		PointCloud& pc = pcit->second;
		 
		const CalibrationPose porigin{Pose{}, -1}; display(porigin);
		
		ref.transform(extrinsics);
		inplace_minmax_filter_depth(ref, 400., 1500., Axis::Z);
		
		if (yes_no_question("Save as .csv"))
		{
			std::ofstream ofs(
				"pts-"+std::to_string(frame)+"-"+std::to_string(getpid())+".csv"
			); 
			if (not ofs.good()) throw std::runtime_error(std::string("Cannot open file pts.csv"));
			
			std::ostringstream headercsv;
			headercsv << "x,y,z\n";
			
			ofs << headercsv.str();
			
			std::ostringstream oss;
			
			for (const auto& f : ref)
			{
				oss << f.x() << "," << f.y() << "," << f.z() << "\n";			
			}
			
			ofs << oss.str();
			ofs.close();
		}
		
		inplace_maxcount_filter_depth(ref, maxcount);
		DEBUG_VAR(ref.size());
		
		PRINT_INFO("=== Displaying pointcloud ("<< frame <<")");
		display(frame, ref);	
			
		auto chamfer_distance = [](const PointCloud& ref, const PointCloud& reading) -> double {
			return (
				0. + //ref.distance(reading, PointCloud::DistanceType::Chamfer) 
				+ reading.distance(ref, PointCloud::DistanceType::Chamfer)
			);
		};
		
		PRINT_DEBUG("=== Starting computing Chamfer distance....");
	Chrono::tic();
		const double score = chamfer_distance(ref, pc);
	Chrono::tac();
		PRINT_DEBUG("...Finished!");
	
		PRINT_INFO("(frame = "<< frame << ") D(ref, reading) = "<< score << " (computed in "<< Chrono::get() << " s)!");	
		
		
		wait();		
	}
	
//FORCE_GUI(false);		
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}


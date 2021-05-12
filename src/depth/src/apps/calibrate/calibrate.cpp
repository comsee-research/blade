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
#include "geometry/depth/RawDepthMap.h"
#include "geometry/depth/PointCloud.h"
#include "geometry/depth/convert.h"

//processing
#include <pleno/processing/calibration/init.h> 
#include <pleno/processing/imgproc/improcess.h> //devignetting
#include "processing/depth/depth.h"
#include "processing/depth/strategy.h"
#include "processing/depth/initialization.h"

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/observations.h>
#include <pleno/io/cfg/poses.h>
#include "io/cfg/depthmaps.h"

#include <pleno/io/images.h>

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
	std::map<Index, Image> pictures;
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
	std::map<Index, BAPObservations> observations;
	for(const auto& ob : bap_obs) observations[ob.frame].push_back(ob);

	DEBUG_VAR(observations.size());
	
////////////////////////////////////////////////////////////////////////////////
// 5) Load depth maps
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("5) Load depth maps");	
	DepthMapsConfig cfg_dms;
	v::load(config.path.dm, cfg_dms);
	
	std::map<Index, RawDepthMap> depthmaps;
	
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
	//for each depth map
	for (const auto& [frame, dm] : depthmaps)
	{
		if (frame == 0) continue;
		
		//get image
		const Image image = pictures[frame];
				
		//split observations according to cluster index
		std::map<Index /* cluster index */, BAPObservations> clusters;
		for(const auto& ob : observations[frame]) clusters[ob.cluster].push_back(ob);	
		
		std::map<Index /* cluster index */, P3D> centroids; 
		
		//for each cluster
		for(auto & [cluster, obs] : clusters)
		{
			//compute reprojected point 
			P3D centroid = P3D::Zero(); 
			double n = 0.;
			
			//for each observation
			for (const auto& ob : obs)
			{
				//get u,v
				const double u = std::floor(ob.u);
				const double v = std::floor(ob.v);
				
				//get depth
				double depth = dm.is_refined_map() ? dm.depth(u,v) : dm.depth(ob.k, ob.l);
				if (depth == DepthInfo::NO_DEPTH) continue;
				if (dm.is_virtual_depth()) depth = mfpc.v2obj(depth, ob.k, ob.l);				
				
				//get pixel
				const P2D pixel = P2D{ob.u, ob.v};
				
				//get ml indexes
				const P2D kl = mfpc.mi2ml(ob.k, ob.l); 
					
				//raytrace
				Ray3D ray; //in CAMERA frame
				if (mfpc.raytrace(pixel, kl[0], kl[1], ray))
				{
					//get depth plane
					PlaneCoefficients plane; plane << 0., 0., 1., -depth;
					
					//get position
					const P3D point = line_plane_intersection(plane, ray);
				
					//accumulate
					centroid += point; ++n;
				}			
			}
			
			if (n != 0.) centroid /= n;
			
			PRINT_DEBUG("Frame ("<< frame <<"), node ("<< cluster<<") = " << scene.node(cluster).transpose() << ", centroid = " << centroid.transpose());
			centroids[cluster] = std::move(centroid);					
		}
		
		wait();
		
		//compute distances and errors
		RMSE rmse{0., 0ul};
		
		for (auto it = centroids.cbegin(); it != centroids.cend(); ++it)
		{
			for (auto nit = std::next(it, 1); nit != centroids.cend(); ++nit)	
			{
				const double ref = (scene.node(it->first) - scene.node(nit->first)).norm();
				const double dist = (it->second - nit->second).norm();
				
				DEBUG_VAR(ref); DEBUG_VAR(dist);
				rmse.add(ref-dist);							
			}
		}
		
		PRINT_DEBUG("Error of frame ("<< frame <<") = " << rmse.get());			
		
		wait();	
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}


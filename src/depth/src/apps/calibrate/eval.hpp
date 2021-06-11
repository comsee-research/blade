#pragma once

#include <iostream>
#include <map>
#include <unistd.h>
#include <unordered_map>

#include <pleno/types.h>

//geometry
#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/object/checkerboard.h>
#include <pleno/geometry/observation.h>
#include "geometry/depth/RawDepthMap.h"

#include "processing/tools/functions.h"

#include <pleno/graphic/gui.h>
#include <pleno/graphic/viewer_2d.h>
#include <pleno/graphic/viewer_3d.h>
#include "../../graphic/display.h"

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

#include "geometry/depth/PointCloud.h"
#include "geometry/depth/convert.h"

#include <pleno/processing/tools/error.h>

void evaluate_scale_error(
	const PlenopticCamera& mfpc, const auto& scaling,
	const CheckerBoard& scene,
	const std::unordered_map<Index, RawDepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations,
	const std::unordered_map<Index, Image>& pictures
) 
{
	constexpr double epsilon = 1e-3;
	v::Palette<int> palette;
	
	std::ofstream ofs(
		"rescaled-error-" + std::to_string(getpid())+".csv"
	); 
	if (not ofs.good()) throw std::runtime_error(std::string("Cannot open file scale-error.csv"));
	
	std::ostringstream headercsv;
	headercsv << "f,v,z,mae,mbe,rmse\n";
	
	ofs << headercsv.str();
	std::ostringstream oss;

	//for each depth map
	for (const auto& [frame, dm] : depthmaps)
	{
		//get image
		Image image = pictures.at(frame);
		
		//convert to pc
		const RawDepthMap mdm = dm.to_metric(mfpc);
		const PointCloud pc = to_pointcloud(mdm, mfpc, image);
		
GUI(		
		display(frame, pc);		
		
		RENDER_DEBUG_2D(
			Viewer::context().layer(Viewer::layer()).name("Frame f = "+std::to_string(frame)), 
			image
		);
		Viewer::update();
);			
		//split observations according to cluster index
		std::map<Index /* cluster index */, BAPObservations> clusters;
		BAPObservations obs; 
		try {
			obs = observations.at(frame);
		} 
		catch (const std::out_of_range& e) {
			PRINT_WARN("No observations for frame ("<< frame << ")");
			continue;
		}
		for(const auto& ob : obs) clusters[ob.cluster].push_back(ob);	
		
		std::map<Index /* cluster index */, P3D> centroids; 
		
		PRINT_INFO("Reprojecting centroid from observations and depthmap...");
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
				if (std::fabs(depth) < DepthInfo::NO_DEPTH + epsilon) continue;
				if (dm.is_virtual_depth()) depth = mfpc.v2obj(depth, ob.k, ob.l);				
				
				depth = scaling(depth);
				
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
			
			if (n != 0.) 
			{
				centroid /= n;
				
				PRINT_DEBUG("Frame ("<< frame <<"), node ("<< cluster<<") = " << scene.node(cluster).transpose() << ", centroid = " << centroid.transpose());
				centroids[cluster] = std::move(centroid);	
			}				
		}
		
		wait();		
	
		//compute distances and errors
		RMSE rmse{0., 0ul};
		MAE	 mae{0., 0ul};
		MBE	 mbe{0., 0ul};
		
		PRINT_INFO("Computing error...");
		for (auto it = centroids.cbegin(); it != centroids.cend(); ++it)
		{
GUI(
			BAPObservations obs;
			mfpc.project(it->second, obs);
			for (const auto& o : obs)
			{
				RENDER_DEBUG_2D(
		  			Viewer::context().layer(Viewer::layer())
		  				.name("Reprojected BAP ("+std::to_string(frame)+")")
						.point_style(v::Cross)
		  				.pen_color(palette[o.cluster+1]).pen_width(2),
		  			Disk{P2D{o.u, o.v}, o.rho}
				);	
			}	
				
			RENDER_DEBUG_3D(
				Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D))
					.name("Centroid"),
				it->second,	5.
		  	);
);			
			for (auto nit = std::next(it, 1); nit != centroids.cend(); ++nit)	
			{
				const double ref = (scene.node(it->first) - scene.node(nit->first)).norm();
				const double dist = (it->second - nit->second).norm();
				
				DEBUG_VAR(ref); DEBUG_VAR(dist);
				rmse.add(100. * (ref-dist) / ref);		
				mae.add(100. * (ref-dist) / ref);	
				mbe.add(100. * (ref-dist) / ref);						
			}
		}
		
		if (centroids.cbegin() != centroids.cend())
		{
			const double z = std::accumulate(centroids.cbegin(), centroids.cend(), 0., 
					[](double tz, const auto& c) { return tz + c.second.z(); }
				) / centroids.size();
				
			oss << frame << "," << mfpc.obj2v(z) << "," << z << "," << mae.get() << "," << mbe.get() << "," << rmse.get() << "\n";			
			
GUI(	
			Viewer::update(Viewer::Mode::m2D);
			Viewer::update(Viewer::Mode::m3D);
);
			PRINT_INFO("Error of frame ("<< frame <<") = " << rmse.get() << " (rmse), " << mae.get() << " (mae), " << mbe.get() << " (mbe)");
		}
		else
		{
			PRINT_WARN("Can't compute error for frame ("<< frame << ")");
		}			
		
		wait();	
	}
	
	ofs << oss.str();		
	ofs.close();
}

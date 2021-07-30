#pragma once

#include <pleno/processing/calibration/link.h>

#include <pleno/geometry/object/constellation.h>

template<typename Observations>
void link_cluster_to_point_constellation_index(
	Observations& observations,
	const PlenopticCamera& pcm, 
	const PointsConstellation& constellation,	
	const Image& gray
)
{
	PRINT_WARN("Link requires manual intervention.");
	
	std::unordered_map<int /* old id */, int /* new id */> id_mapping;
	
FORCE_GUI(true);
	const double ratio = double(gray.rows) / double(gray.cols);
	const int base_size = 800;
	
	GUI(
		RENDER_DEBUG_2D(
			Viewer::context().size(base_size,base_size*ratio).layer(Viewer::layer()).name("Scene"), 
			gray
		);
    );
    Viewer::update();	
	
	//compute barycenters
	const Observations barycenters = compute_barycenters(observations); DEBUG_VAR(barycenters.size());
	display(-1, barycenters, tag::Barycenters{});
	
	//for each point in constellation
	for (int i = 0; i < constellation.size(); ++i)
	{
		volatile bool finished = false;
		
		P2D point; 
		
		PRINT_INFO("Click on cluster corresponding to point (" << i << ") in constellation");	
		Viewer::context().on_click([&](float x, float y){
			if (x > 0. and x < pcm.sensor().width() and y > 0. and y < pcm.sensor().height())
			{
				PRINT_DEBUG("Click at position ("<< x << ", "<< y << ")");
				GUI(
					P2D c = P2D{x,y};
		  			Viewer::context().layer(Viewer::layer())
		  				.name("Selected cluster")
		  				.pen_color(v::purple).pen_width(5)
		  				.add_circle(c[0], c[1], 25.)
		  				.update();
				);
			
				point.x() = x;
				point.y() = y;			
				finished = true;
			}
			else
			{
				PRINT_ERR("Click out of bound, unvalidated. Try again.");
				return;
			}
		});
		
		//wait for click	
		while(not finished);
		Viewer::context().on_click([&](float x, float y){});
		
		//find nearest cluster
		int minc = -1; double mindist = 1e5; int j = 0;
		for (const auto& center : barycenters)
		{
			const double dist = (P2D{center.u, center.v} - point).norm(); 
			if (dist < mindist)
			{
				minc = j;
				mindist = dist;
			}
			++j;
		}
		
		id_mapping[minc] = i;							
	}
	
	//assign new cluster id
	for (auto& o : observations)
	{
		if(id_mapping.count(o.cluster) > 0) 
		{
			o.cluster = id_mapping[o.cluster];
			o.isValid = true;	
		}
		else
		{
			o.cluster = -1;
			o.isValid = false;		
		}
	}

FORCE_GUI(false);
}



#include "depth.h"

#include <thread> //std::thread

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

#include <pleno/graphic/display.h>

#include <pleno/processing/tools/stats.h> //median

#include "geometry/depth/depthmap.h"
#include "processing/tools/chrono.h"

#include "../../graphic/display.h"

#include "strategy.h"
#include "filter.h"
#include "initialization.h"
#include "compute.h"

#define	DISPLAY_FRAME					1

//******************************************************************************
//******************************************************************************
//******************************************************************************
void estimate_depth(
	DepthMap& depthmap,
	const PlenopticCamera& mfpc,
	const Image& img,
	const DepthEstimationStrategy& strategies,
	const Image& color,
	bool gui
)
{	
	std::string ss = "";
	if (strategies.mtype == DepthMap::MapType::REFINED) ss = "refined ";
	else if (strategies.probabilistic) ss = "probabilistic ";
	
	PRINT_INFO("=== Start " << ss <<"depth estimation" << (mfpc.multifocus() ? " (BLADE)": " (DISP)"));	
#if DISPLAY_FRAME
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name("Frame"),
		img
  	);
#endif	
//------------------------------------------------------------------------------
	DepthMap dm{depthmap};
	
	const unsigned int nbthreads = strategies.multithread ?
			(strategies.nbthread == -1 ?  std::thread::hardware_concurrency()-1 : strategies.nbthread)
		: 	1;
		
//------------------------------------------------------------------------------
	Chrono::tic();
	// Run depth estimation
	std::vector<std::thread> threads;
	for(unsigned int i=0; i< nbthreads; ++i)
	{
		PRINT_DEBUG("Running estimation on thread (" << i <<")...");

		const auto [k,l] = initialize_kl(i, nbthreads, mfpc.mia(), strategies.init);
		
		if (strategies.mtype == DepthMap::MapType::REFINED)
		{
			threads.push_back(
				std::thread(
					compute_refined_depthmap, 
					std::ref(dm), std::cref(mfpc), std::cref(img), k, l,
					std::cref(strategies)
				)
			);
		}
		else if (strategies.probabilistic)
		{
			threads.push_back(
				std::thread(
					compute_probabilistic_depthmap, 
					std::ref(dm), std::cref(mfpc), std::cref(img), k, l,
					std::cref(strategies)
				)
			);
		}
		else
		{
			threads.push_back(
				std::thread(
					compute_depthmap, 
					std::ref(dm), std::cref(mfpc), std::cref(img), k, l,
					std::cref(strategies)
				)
			);
		}
	}		
//------------------------------------------------------------------------------	
	// Wait for all thread to finish
	for (std::thread & t : threads)
		if (t.joinable()) t.join();
		
	Chrono::tac();
	
	PRINT_INFO("=== Estimation finished (in "<< Chrono::get() << " s)! Displaying depth map...");	

	auto reduce = [](const DepthMap& dm) -> double {
		std::vector<double> zs; zs.reserve(dm.width() * dm.height());		
		for (std::size_t k = 0; k < dm.width(); ++k)
			for (std::size_t l = 0; l < dm.height(); ++l)
				if (dm.depth(k,l) != DepthInfo::NO_DEPTH)
					zs.emplace_back(dm.depth(k,l));
					
		zs.shrink_to_fit();
		
		return median(zs);
	};

	dm.copy_to(depthmap);
	if (gui)
	{
	GUI(
		display(depthmap, mfpc);	
	//------------------------------------------------------------------------------	
	#if 0
		// Filtered depth map
		PRINT_INFO("=== Filtering depth map...");	
	#if 0
		inplace_minmax_filter_depth(dm, 2., 15.);	
		DepthMap filtereddm = median_filter_depth(dm, 4.1);
		PRINT_INFO("=== Filtering finished! Displaying depth map...");
		display(filtereddm);
	#else
		DepthMap filtereddm = minmax_filter_depth(dm, 2., 15.);
		#if 0
			const DepthMap edm = morph_erosion_filter_depth(filtereddm);
			const DepthMap ddm = morph_dilation_filter_depth(filtereddm);
			const DepthMap odm = morph_opening_filter_depth(filtereddm);
			const DepthMap cdm = morph_closing_filter_depth(filtereddm);
			const DepthMap sdm = morph_smoothing_filter_depth(filtereddm);
			const DepthMap dytdm = morph_dyt_filter_depth(filtereddm);
			const DepthMap tetdm = morph_tet_filter_depth(filtereddm);
			const DepthMap occodm = morph_occo_filter_depth(filtereddm);
		#endif
		PRINT_INFO("=== Filtering finished! Displaying depth map...");
		display(filtereddm);
		#if 0
			display(edm);
			display(ddm);
			display(odm);
			display(cdm);
			display(sdm);
			display(dytdm);
			display(tetdm);
			display(occodm);
			
			wait();
		#endif
	#endif
		#endif
	//------------------------------------------------------------------------------	
		// Convert to metric depth map
		PRINT_INFO("=== Converting depth map...");	
		DepthMap mdm = dm.to_metric(mfpc);
		PRINT_INFO("=== Conversion finished! Displaying metric depth map...");	
		display(mdm, mfpc);	
		
		PRINT_WARN("Estimated virtual depth = "<< reduce(dm));
		PRINT_WARN("Estimated metric depth = "<< reduce(mdm));
		
		PRINT_INFO("=== Converting to pointcloud...");
		//inplace_median_filter_depth(mdm, mfpc.mia(), mdm.is_coarse_map() ? 3. : 2.);
		PointCloud pc = PointCloud{mdm, mfpc, color};
		
		PRINT_INFO("=== Conversion finished! Displaying pointcloud (" << pc.size() << ")...");	
		display(mfpc);
		display(0, pc);	

		wait();
	);
	}
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void estimate_depth_from_obs(
	DepthMap& depthmap,
	const PlenopticCamera& mfpc,
	const Image& img,
	const BAPObservations& observations, /*  (u,v,rho) */
	bool gui
)
{	
	PRINT_INFO("=== Start depth estimation from observations");	
#if DISPLAY_FRAME
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name("Frame"),
		img
  	);
#endif	
//------------------------------------------------------------------------------
	DepthMap dm{depthmap};
	
//------------------------------------------------------------------------------
	Chrono::tic();	
	// Run depth estimation
	compute_depthmap_from_obs(
		dm, mfpc, img, observations
	);

//------------------------------------------------------------------------------	
	Chrono::tac();
	PRINT_INFO("=== Estimation finished (in "<< Chrono::get() << " s)! Displaying depth map...");	
	if (gui)
	{
		display(dm, mfpc);
		
	//------------------------------------------------------------------------------	
		// Convert to metric depth map
		PRINT_INFO("=== Converting depth map...");	
		DepthMap mdm = dm.to_metric(mfpc);
		PRINT_INFO("=== Conversion finished! Displaying metric depth map...");	
		display(mdm, mfpc);	
		
		wait();
	}
	dm.copy_to(depthmap);
}

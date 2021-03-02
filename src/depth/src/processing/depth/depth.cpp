#include "depth.h"

#include <thread> //std::thread

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

#include <pleno/graphic/display.h>

#include "geometry/depth/RawDepthMap.h"
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
	RawDepthMap& depthmap,
	const PlenopticCamera& mfpc,
	const Image& img,
	const DepthEstimationStrategy& strategies
)
{	
	std::string ss = "";
	if (strategies.mtype == RawDepthMap::MapType::DENSE) ss = "dense";
	else if (strategies.probabilistic) ss = "probabilistic";
	
	PRINT_INFO("=== Start "+ss+" depth estimation");	
#if DISPLAY_FRAME
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name("Frame"),
		img
  	);
#endif	
//------------------------------------------------------------------------------
	RawDepthMap dm{depthmap};
	
	const unsigned int nbthreads = 
		(strategies.multithread and strategies.nbthread == -1) ? 
			std::thread::hardware_concurrency()-1 
		: 	strategies.nbthread;
		
//------------------------------------------------------------------------------
	auto t_start = std::chrono::high_resolution_clock::now();	
	// Run depth estimation
	std::vector<std::thread> threads;
	for(unsigned int i=0; i< nbthreads; ++i)
	{
		PRINT_DEBUG("Running estimation on thread (" << i <<")...");

		const auto [k,l] = initialize_kl(i, nbthreads, mfpc.mia(), strategies.init);
		
		if (strategies.mtype == RawDepthMap::MapType::DENSE)
		{
			threads.push_back(
				std::thread(
					compute_dense_depthmap, 
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
		
	auto t_end = std::chrono::high_resolution_clock::now();
	const double computational_time = std::chrono::duration<double>(t_end-t_start).count();
	
	PRINT_INFO("=== Estimation finished (in "<< computational_time << " s)! Displaying depth map...");	

	dm.copy_to(depthmap);
	display(depthmap);
	
//------------------------------------------------------------------------------	
#if 0
	// Filtered depth map
	PRINT_INFO("=== Filtering depth map...");	
#if 0
	inplace_minmax_filter_depth(dm, 2., 15.);	
	RawDepthMap filtereddm = median_filter_depth(dm, 4.1);
	PRINT_INFO("=== Filtering finished! Displaying depth map...");
	display(filtereddm);
#else
	RawDepthMap filtereddm = minmax_filter_depth(dm, 2., 15.);
	#if 0
		const RawDepthMap edm = morph_erosion_filter_depth(filtereddm);
		const RawDepthMap ddm = morph_dilation_filter_depth(filtereddm);
		const RawDepthMap odm = morph_opening_filter_depth(filtereddm);
		const RawDepthMap cdm = morph_closing_filter_depth(filtereddm);
		const RawDepthMap sdm = morph_smoothing_filter_depth(filtereddm);
		const RawDepthMap dytdm = morph_dyt_filter_depth(filtereddm);
		const RawDepthMap tetdm = morph_tet_filter_depth(filtereddm);
		const RawDepthMap occodm = morph_occo_filter_depth(filtereddm);
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
	RawDepthMap mdm = dm.to_metric(mfpc);
	PRINT_INFO("=== Conversion finished! Displaying metric depth map...");	
	display(mdm);	
	
	wait();
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void estimate_depth_from_obs(
	RawDepthMap& depthmap,
	const PlenopticCamera& mfpc,
	const Image& img,
	const BAPObservations& observations /*  (u,v,rho) */
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
	RawDepthMap dm{depthmap};
	
//------------------------------------------------------------------------------
	auto t_start = std::chrono::high_resolution_clock::now();	
	// Run depth estimation
	compute_depthmap_from_obs(
		dm, mfpc, img, observations
	);

//------------------------------------------------------------------------------	
	auto t_end = std::chrono::high_resolution_clock::now();
	const double computational_time = std::chrono::duration<double>(t_end-t_start).count();
	
	PRINT_INFO("=== Estimation finished (in "<< computational_time << " s)! Displaying depth map...");	
	display(dm);
	
//------------------------------------------------------------------------------	
	// Convert to metric depth map
	PRINT_INFO("=== Converting depth map...");	
	RawDepthMap mdm = dm.to_metric(mfpc);
	PRINT_INFO("=== Conversion finished! Displaying metric depth map...");	
	display(mdm);	
	
	wait();
	
	dm.copy_to(depthmap);
}

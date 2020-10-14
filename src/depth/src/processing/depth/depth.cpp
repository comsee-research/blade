#include "depth.h"

#include <thread> //std::thread

#include <pleno/processing/improcess.h>

#include <pleno/processing/tools/lens.h>
#include <pleno/processing/tools/rmse.h>
#include <pleno/processing/tools/stats.h>

#include <pleno/io/printer.h>
#include "io/choice.h"

#include <pleno/graphic/display.h>

#include "geometry/depth/RawCoarseDepthMap.h"
#include "../../graphic/display.h"

#include "strategy.h"
#include "filter.h"
#include "initialization.h"
#include "compute.h"

#define	DISPLAY_FRAME					0
#define ENABLE_MULTI_THREAD 			1
#define USE_EROSION_FILTER				0

//******************************************************************************
//******************************************************************************
//******************************************************************************
void estimate_depth(
	RawCoarseDepthMap& rcdm,
	const PlenopticCamera& mfpc,
	const Image& img,
	const DepthEstimationStrategy& strategies
)
{	
	PRINT_INFO("=== Start depth estimation");	
#if DISPLAY_FRAME
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name("Frame"),
		img
  	);
#endif	
//------------------------------------------------------------------------------
	RawCoarseDepthMap dm{rcdm};
	
#if ENABLE_MULTI_THREAD
	const unsigned int nthreads = std::thread::hardware_concurrency()-1;	
#else
	const unsigned int nthreads = 1;
#endif
//------------------------------------------------------------------------------
	auto t_start = std::chrono::high_resolution_clock::now();	
	// Run depth estimation
	std::vector<std::thread> threads;
	for(unsigned int i=0; i< nthreads; ++i)
	{
		PRINT_DEBUG("Running estimation on thread (" << i <<")...");

		const auto [k,l] = initialize_kl(i, nthreads, mfpc.mia(), strategies.init);

		threads.push_back(
			std::thread(
				compute_depthmap, 
				std::ref(dm), std::cref(mfpc), std::cref(img), k, l,
				strategies
			)
		);
	}		
//------------------------------------------------------------------------------	
	// Wait for all thread to finish
	for (std::thread & t : threads)
		if (t.joinable()) t.join();
		
	auto t_end = std::chrono::high_resolution_clock::now();
	const double computational_time = std::chrono::duration<double>(t_end-t_start).count();
	
	PRINT_INFO("=== Estimation finished (in "<< computational_time << " s)! Displaying depth map...");	
	display(dm);
	
//------------------------------------------------------------------------------	
	// Filtered depth map
	PRINT_INFO("=== Filtering depth map...");	
#if USE_EROSION_FILTER
	RawCoarseDepthMap erodeddm = erosion_filter_depth(dm);
	RawCoarseDepthMap filtereddm = median_filter_depth(erodeddm, 4.1);
#else
	RawCoarseDepthMap filtereddm = median_filter_depth(dm, 4.1);
#endif
	PRINT_INFO("=== Filtering finished! Displaying depth map...");
#if USE_EROSION_FILTER
	display(erodeddm);		
#endif
	display(filtereddm);

//------------------------------------------------------------------------------	
	// Convert to metric depth map
	PRINT_INFO("=== Converting depth map...");	
	RawCoarseDepthMap mdm = filtereddm.as_metric();
	PRINT_INFO("=== Conversion finished! Displaying metric depth map...");	
	display(mdm);	
	
	wait();
	
	rcdm.copy_map(filtereddm);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void estimate_probabilistic_depth(
	RawCoarseDepthMap& rcdm, RawCoarseDepthMap& confidencedm,
	const PlenopticCamera& mfpc,
	const Image& img,
	const DepthEstimationStrategy& strategies
)
{	
	PRINT_INFO("=== Start probabilistic depth estimation");	
#if DISPLAY_FRAME
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name("Frame"),
		img
  	);
#endif	
//------------------------------------------------------------------------------
	RawCoarseDepthMap dm{rcdm};
	
#if ENABLE_MULTI_THREAD
	const unsigned int nthreads = std::thread::hardware_concurrency()-1;	
#else
	const unsigned int nthreads = 1;
#endif
//------------------------------------------------------------------------------
	auto t_start = std::chrono::high_resolution_clock::now();	
	// Run depth estimation
	std::vector<std::thread> threads;
	for(unsigned int i=0; i< nthreads; ++i)
	{
		PRINT_DEBUG("Running estimation on thread (" << i <<")...");

		const auto [k,l] = initialize_kl(i, nthreads, mfpc.mia(), strategies.init);
		
		threads.push_back(
			std::thread(
				compute_probabilistic_depthmap,
				std::ref(dm), std::ref(confidencedm),
				std::cref(mfpc), std::cref(img), k, l,
				strategies
			)
		);
	}		
//------------------------------------------------------------------------------	
	// Wait for all thread to finish
	for (std::thread & t : threads)
		if (t.joinable()) t.join();
		
	auto t_end = std::chrono::high_resolution_clock::now();
	const double computational_time = std::chrono::duration<double>(t_end-t_start).count();
	
	PRINT_INFO("=== Estimation finished (in "<< computational_time << " s)! Displaying depth map...");	
	
	rcdm.copy_map(dm);
	display(dm);
	
	
//------------------------------------------------------------------------------	
	// Filtered depth map
	PRINT_INFO("=== Filtering depth map...");	
#if USE_EROSION_FILTER
	RawCoarseDepthMap erodeddm = erosion_filter_depth(dm);
	RawCoarseDepthMap filtereddm = median_filter_depth(erodeddm); //, 4.1);
#else
	RawCoarseDepthMap filtereddm = median_filter_depth(dm); //, 4.1);
#endif
	PRINT_INFO("=== Filtering finished! Displaying depth map...");
	
#if USE_EROSION_FILTER
	display(erodeddm);		
#endif
	display(filtereddm);
	
	PRINT_INFO("=== Displaying confidence map...");
	display(confidencedm);
	
//------------------------------------------------------------------------------	
	// Convert to metric depth map
	PRINT_INFO("=== Converting depth map...");	
	RawCoarseDepthMap mdm = filtereddm.as_metric();
	PRINT_INFO("=== Conversion finished! Displaying metric depth map...");	
	display(mdm);	
	
	wait();
	rcdm.copy_map(filtereddm);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void estimate_depth_from_obs(
	RawCoarseDepthMap& rcdm,
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
	RawCoarseDepthMap dm{rcdm};
	
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
	RawCoarseDepthMap mdm = dm.as_metric();
	PRINT_INFO("=== Conversion finished! Displaying metric depth map...");	
	display(mdm);	
	
	wait();
	
	rcdm.copy_map(dm);
}

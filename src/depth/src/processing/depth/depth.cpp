#include "depth.h"

#include <thread> //std::thread

#include <pleno/processing/imgproc/improcess.h>

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
	rcdm.copy_map(dm);
	display(rcdm);
	
//------------------------------------------------------------------------------	
#if 0
	// Filtered depth map
	PRINT_INFO("=== Filtering depth map...");	
#if 0
	inplace_minmax_filter_depth(dm, 2., 15.);	
	RawCoarseDepthMap filtereddm = median_filter_depth(dm, 4.1);
	PRINT_INFO("=== Filtering finished! Displaying depth map...");
	display(filtereddm);
#else
	RawCoarseDepthMap filtereddm = minmax_filter_depth(dm, 2., 15.);
	#if 0
		const RawCoarseDepthMap edm = morph_erosion_filter_depth(filtereddm);
		const RawCoarseDepthMap ddm = morph_dilation_filter_depth(filtereddm);
		const RawCoarseDepthMap odm = morph_opening_filter_depth(filtereddm);
		const RawCoarseDepthMap cdm = morph_closing_filter_depth(filtereddm);
		const RawCoarseDepthMap sdm = morph_smoothing_filter_depth(filtereddm);
		const RawCoarseDepthMap dytdm = morph_dyt_filter_depth(filtereddm);
		const RawCoarseDepthMap tetdm = morph_tet_filter_depth(filtereddm);
		const RawCoarseDepthMap occodm = morph_occo_filter_depth(filtereddm);
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
	RawCoarseDepthMap mdm = dm.as_metric();
	PRINT_INFO("=== Conversion finished! Displaying metric depth map...");	
	display(mdm);	
	
	wait();
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
	display(rcdm);
		
//------------------------------------------------------------------------------	
#if 0
	// Filtered depth map
	PRINT_INFO("=== Filtering depth map...");
#if 0
		inplace_minmax_filter_depth(dm, 2., 15.);	
		RawCoarseDepthMap filtereddm = median_filter_depth(dm); //, 4.1);
		PRINT_INFO("=== Filtering finished! Displaying depth map...");
		display(filtereddm);
		
		PRINT_INFO("=== Displaying confidence map...");
		display(confidencedm);
#else
	RawCoarseDepthMap filtereddm = minmax_filter_depth(dm, 2., 15.);
	#if 0
		const RawCoarseDepthMap edm = morph_erosion_filter_depth(filtereddm);
		const RawCoarseDepthMap ddm = morph_dilation_filter_depth(filtereddm);
		const RawCoarseDepthMap odm = morph_opening_filter_depth(filtereddm);
		const RawCoarseDepthMap cdm = morph_closing_filter_depth(filtereddm);
		const RawCoarseDepthMap sdm = morph_smoothing_filter_depth(filtereddm);
		const RawCoarseDepthMap dytdm = morph_dyt_filter_depth(filtereddm);
		const RawCoarseDepthMap tetdm = morph_tet_filter_depth(filtereddm);
		const RawCoarseDepthMap occodm = morph_occo_filter_depth(filtereddm);
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
	RawCoarseDepthMap mdm = dm.as_metric();
	PRINT_INFO("=== Conversion finished! Displaying metric depth map...");	
	display(mdm);	
	
	wait();
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

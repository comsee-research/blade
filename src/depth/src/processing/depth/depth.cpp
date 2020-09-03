#include "depth.h"

#include <thread> //std::thread
#include <variant> //std::variant

//optimization
#include "optimization/depth.h"
#include "optimization/errors/blurawaredisp.h" //BlurAwareDisparityCostError
#include "optimization/errors/disparity.h" //DisparityCostError

#include <pleno/geometry/mia.h> //MicroImage

#include <pleno/processing/improcess.h>

#include <pleno/processing/tools/lens.h>
#include <pleno/processing/tools/rmse.h>
#include <pleno/processing/tools/stats.h>

#include <pleno/io/printer.h>
#include <pleno/graphic/display.h>

#include "geometry/depth/RawCoarseDepthMap.h"
#include "../../graphic/display.h"

#include "strategy.h"
#include "neighbors.h"
#include "filter.h"
#include "initialization.h"

constexpr double 		AUTOMATIC_LAMBDA_SCALE 	= -1.;

#define	DISPLAY_FRAME	1
#define VERBOSE_OPTIM	0

//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize_depth(
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t ck, std::size_t cl, //current indexes
	double& depth, //in/out
	ObservationsParingStrategy mode = ObservationsParingStrategy::CENTRALIZED
)
{ 	
	const std::size_t I = mfpc.I();
	const int W = std::floor(mfpc.mia().diameter());
	
	const bool useBlur = (I > 0u); 
	
	using SolverBLADE = lma::Solver<BlurAwareDisparityCostError>;
	using SolverDISP = lma::Solver<DisparityCostError>;
	using Solver_t = std::variant<std::monostate, SolverBLADE, SolverDISP>;
	
	Solver_t vsolver;
		if(useBlur) vsolver.emplace<SolverBLADE>(AUTOMATIC_LAMBDA_SCALE, 150, 1.0 - 1e-12);
		else vsolver.emplace<SolverDISP>(AUTOMATIC_LAMBDA_SCALE, 150, 1.0 - 1e-12);  

	//init virtual depth
	VirtualDepth hypothesis;
	hypothesis.v = depth;
	
 	//compute ref observation 	
 	MicroImage ref{
		ck, cl,
		mfpc.mia().nodeInWorld(ck,cl),
		mfpc.mia().radius(),
		lens_type(I, ck, cl)
	};
	
	Image refview;
	cv::getRectSubPix(
		scene, cv::Size{W,W}, 
		cv::Point2d{ref.center[0], ref.center[1]}, refview
	);
	
	std::visit([&](auto&& solver) { using T = std::decay_t<decltype(solver)>;
	if constexpr (not std::is_same_v<T, std::monostate>) 
	{
		using FunctorError_t = typename std::conditional<std::is_same_v<T, SolverBLADE>, 
			BlurAwareDisparityCostError, DisparityCostError>::type;
	
		if(mode == ObservationsParingStrategy::CENTRALIZED)
		{
		 	//for each neighbor, create observation
		 	for(auto [nk, nl] : neighs)
		 	{
		 		MicroImage target{
					nk, nl,
					mfpc.mia().nodeInWorld(nk,nl),
					mfpc.mia().radius(),
					lens_type(I, nk, nl)
				};
				
				Image targetview;
				cv::getRectSubPix(
					scene, cv::Size{W,W},  
					cv::Point2d{target.center[0], target.center[1]}, targetview
				);
				
				//add in solver
				solver.add(
					FunctorError_t{
						refview, targetview,
						ref, target,
						mfpc
					},
					&hypothesis
				);	 	
		 	}
		}	
		else if (mode == ObservationsParingStrategy::ALL_PAIRS)
		{
			std::vector<MicroImage> vmi; vmi.reserve(100);
			std::vector<Image> vview; vview.reserve(100);
			
			vmi.emplace_back(ref); vview.emplace_back(refview);
			
			for(auto [nk, nl] : neighs)
		 	{
		 		MicroImage target{
					nk, nl,
					mfpc.mia().nodeInWorld(nk,nl),
					mfpc.mia().radius(),
					lens_type(I, nk, nl)
				};
				
				Image targetview;
				cv::getRectSubPix(
					scene, cv::Size{W,W},  
					cv::Point2d{target.center[0], target.center[1]}, targetview
				);
				
				vmi.emplace_back(target); vview.emplace_back(targetview);
			}

			//for each observation
			for(std::size_t i = 0; i < vmi.size(); ++i)
			{		
				for(std::size_t j = i+1; j < vmi.size(); ++j)
				{
					//add in solver
					solver.add(
						FunctorError_t{
							vview[i], vview[j],
							vmi[i], vmi[j],
							mfpc
						},
						&hypothesis
					);
				}
			}	
		}
		else
		{
			DEBUG_ASSERT(false, "Can't optimize depth, no other strategy implemented yet");
		}

	#if VERBOSE_OPTIM
	 	PRINT_DEBUG("*** initial depth value = " << hypothesis.v);
	 	solver.solve(lma::DENSE, lma::enable_verbose_output());
	 	PRINT_DEBUG("*** optimized depth value = " << hypothesis.v);
	#else
	 	solver.solve(lma::DENSE); //no verbose, lma::enable_verbose_output());
	#endif
	}}, vsolver);
	
 	depth = hypothesis.v;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap(
	RawCoarseDepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	BeliefPropagationStrategy mode = BeliefPropagationStrategy::NONE
)
{	
	std::queue<IndexPair> microimages;
	microimages.emplace(kinit, linit);
	
	while(not (microimages.empty()))
	{
		auto [ck, cl] = microimages.front(); //current indexes
		microimages.pop();
				
		//Already computed, nothing to do
		if(dm.state(ck,cl) == DepthInfo::State::COMPUTED) continue;
		
		//Compute first hypothesis using inner ring
		if(dm.state(ck,cl) == DepthInfo::State::UNINITIALIZED)
		{
			//get neighbors
		 	std::vector<IndexPair> neighs = neighbors(mfpc.mia(), ck, cl, 4.); 
		 	
		 	double hypothesis = initialize_depth(
		 		neighs, mfpc, scene, 
		 		ck, cl,
		 		dm.min_depth(), dm.max_depth(), 15.
		 	);
		 	
		 	if(not dm.is_valid_depth(hypothesis))
		 	{
		 		dm.depth(ck, cl) = DepthInfo::NO_DEPTH;
		 		dm.state(ck, cl) = DepthInfo::State::COMPUTED;	
		 		
		 		for(auto &n: neighs) microimages.push(n);
		 		
		 		continue;		 	
		 	}
		 	
		 	dm.depth(ck, cl) = hypothesis;
		 	dm.state(ck, cl) = DepthInfo::State::INITIALIZED;				
		}
		
		//Compute depth hypothesis
		if(dm.state(ck,cl) == DepthInfo::State::INITIALIZED)
		{
			double depth = dm.depth(ck, cl);
			std::vector<IndexPair> neighs = neighbors(dm.mia(), ck, cl, depth);
			
			optimize_depth(neighs, mfpc, scene, ck, cl, depth);
			
			if(dm.is_valid_depth(depth)) dm.depth(ck, cl) = depth;
		 	dm.state(ck, cl) = DepthInfo::State::COMPUTED;	
			
			for(auto &n: neighs) 
			{
				//if already computed, do nothing
				if(dm.state(n.k, n.l) == DepthInfo::State::COMPUTED) 
				{
					/* Add strategy to check if estimation is coherent */
					continue;
				}
				
				//if already initialized, average hypotheses
				if(dm.state(n.k, n.l) == DepthInfo::State::INITIALIZED)
				{					
					/* Add strategy to update hypothesis */					
					continue;
				}
				
				//else uninitialized then initalize hypothesis if valid
				if(mode == BeliefPropagationStrategy::ALL_NEIGHS and dm.is_valid_depth(depth))
				{		
					dm.depth(n.k, n.l) = depth;
			 		dm.state(n.k, n.l) = DepthInfo::State::INITIALIZED;			
				}
				//add to queue
				microimages.push(n);
			}
			
			if(mode == BeliefPropagationStrategy::FIRST_RING and dm.is_valid_depth(depth))
			{		
				std::vector<IndexPair> innerneighs = neighbors(dm.mia(), ck, cl, 3.);
				
				for(auto& n: innerneighs)
				{
					if(dm.state(n.k, n.l) == DepthInfo::State::UNINITIALIZED)
					{
						dm.depth(n.k, n.l) = depth;
			 			dm.state(n.k, n.l) = DepthInfo::State::INITIALIZED;	
			 		}
		 		}		
			}	
		}		
	}
	PRINT_DEBUG("Local depth estimation finished.");
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void estimate_depth(
	const PlenopticCamera& mfpc,
	const Image& img
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
	constexpr double nearfocusd = 1e3;
	constexpr double farfocusd = 5e3; 
	
	double maxd, mind, d = mfpc.distance_focus() * 2.;
	if(d < nearfocusd) //short distances
	{
		maxd = mfpc.distance_focus() * 1.2;
		mind = mfpc.v2obj(12.); //std::ceil(mfpc.focal()) * 2.;
	}
	else if (d < farfocusd) //middle distances
	{
		maxd = mfpc.distance_focus() * 2.;
		mind = 6. * std::ceil(mfpc.focal());
	}
	else //far distances
	{
		maxd = farfocusd;
		mind = 8. * std::ceil(mfpc.focal()); 
	}

	RawCoarseDepthMap dm{mfpc, mfpc.obj2v(maxd), mfpc.obj2v(mind)};
	
	BeliefPropagationStrategy mode = BeliefPropagationStrategy::NONE;
	
	const unsigned int nthreads = std::thread::hardware_concurrency()-1;	
//------------------------------------------------------------------------------	
	// Run depth estimation
	std::vector<std::thread> threads;
	for(unsigned int i=0; i< nthreads; ++i)
	{
		PRINT_DEBUG("Running estimation on thread (" << i <<")...");
		//randomly draw microimage indexes
		auto [k,l] = initialize_kl(i, nthreads, mfpc.mia());
		
		threads.push_back(
			std::thread(
				compute_depthmap,
				std::ref(dm), std::cref(mfpc), std::cref(img), k, l, mode
			)
		);
	}		
//------------------------------------------------------------------------------	
	// Wait for all thread to finish
	for (std::thread & t : threads)
		if (t.joinable()) t.join();
	
	PRINT_INFO("=== Estimation finished! Displaying depth map...");	
	display(dm);
	
//------------------------------------------------------------------------------	
	// Filtered depth map
	PRINT_INFO("=== Filtering depth map...");	
	//RawCoarseDepthMap erodeddm = erosion_filter_depth(dm);
	RawCoarseDepthMap filtereddm = median_filter_depth(dm, 4.1);
	PRINT_INFO("=== Filtering finished! Displaying depth map...");
	//display(erodeddm);		
	display(filtereddm);	
	
//------------------------------------------------------------------------------	
	// Convert to metric depth map
	PRINT_INFO("=== Converting depth map...");	
	RawCoarseDepthMap mdm = filtereddm.as_metric();
	PRINT_INFO("=== Conversion finished! Displaying metric depth map...");	
	display(mdm);	
	
	std::getchar();
}

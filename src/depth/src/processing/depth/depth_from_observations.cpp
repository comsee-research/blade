#include "depth.h"

//optimization
#include "optimization/depth.h"
#include "optimization/errors/blurawaredisp.h" //BlurAwareDisparityCostError
#include "optimization/errors/blurequalizationdisp.h" //BlurEqualizationDisparityCostError
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

constexpr double 	AUTOMATIC_LAMBDA_SCALE 	= -1.;

#define EXPORT_COST_FUNCTION 	1
#define DISPLAY_FRAME			0

//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize_depths(
	//OUT
	std::vector<VirtualDepth>& depths, /* */
	//IN
	const PlenopticCamera& mfpc,
	const BAPObservations& observations, /*  (u,v,rho) */
	const std::vector<Image>& images
)
{
	constexpr int W = 24u;
	
	using FunctorError_t = BlurEqualizationDisparityCostError; //BlurAwareDisparityCostError;
	using Solver_t = lma::Solver<FunctorError_t>;
	
	//split observations according to frame index
	std::unordered_map<Index /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);	
	
	//for each frame
	for(auto & [frame, baps]: obs)
	{ 		
		if (frame != 7) continue; //frame != 8 and 
		
		PRINT_INFO("Estimation for frame f = " << frame); //<< ", cluster = " << cluster);
#if DISPLAY_FRAME
		RENDER_DEBUG_2D(
			Viewer::context().layer(Viewer::layer()++)
  				.name("Frame f = "+std::to_string(frame)),
  			images[frame]
	  	);
#endif	
#if EXPORT_COST_FUNCTION
		std::vector<FunctorError_t> functors; functors.reserve(20456);
#endif
		Solver_t solver{AUTOMATIC_LAMBDA_SCALE, 50, 1.0 - 1e-13};
		
		VirtualDepth depth; depth.v = depths[frame].v;
		
		//split observations according to cluster index
		std::unordered_map<Index /* cluster index */, BAPObservations> clusters;
		for(const auto& ob : baps)
			clusters[ob.cluster].push_back(ob);	
		
		//for each cluster
		for(auto & [cluster, obs_] : clusters)
		{		
			//for each observation
			for(std::size_t i = 0; i < obs_.size() ; ++i)
			{
				auto current = obs_.begin()+i;

				std::for_each( current+1, obs_.end(), 
					[lhs=*current, &mfpc, &W, img=images[frame], &depth, &solver
					#if EXPORT_COST_FUNCTION
						, &functors
					#endif
						](const auto &rhs) -> void {
						const std::size_t I = mfpc.I();
						
						// get micro-images
						MicroImage mii{
							static_cast<std::size_t>(lhs.k), static_cast<std::size_t>(lhs.l),
							mfpc.mia().nodeInWorld(lhs.k,lhs.l),
							W/2.,
							lens_type(I, lhs.k,lhs.l)
						};
						
						MicroImage mij{
							static_cast<std::size_t>(rhs.k), static_cast<std::size_t>(rhs.l),
							mfpc.mia().nodeInWorld(rhs.k,rhs.l),
							W/2.,
							lens_type(I, rhs.k, rhs.l)
						};
						
						//extract images
						Image viewi, viewj;
						cv::getRectSubPix(img, cv::Size{W,W}, cv::Point2d{mii.center[0], mii.center[1]}, viewi);
						cv::getRectSubPix(img, cv::Size{W,W}, cv::Point2d{mij.center[0], mij.center[1]}, viewj);
						
						//add in solver
						solver.add(
							FunctorError_t{
								viewi, viewj,
								mii, mij,
								mfpc
							},
							&depth
						);
					#if EXPORT_COST_FUNCTION	
						functors.emplace_back(
							FunctorError_t{
								viewi, viewj,
								mii, mij,
								mfpc
							}
						);	
					#endif				
					}
				);
			}//pair of observations
		
		}//clusters

		solver.solve(lma::DENSE, lma::enable_verbose_output());
		const double optimized_depth = depth.v; 
		PRINT_INFO("Optimized depth for frame ("<< frame<<"), v = " << optimized_depth << ", z = " << mfpc.v2obj(optimized_depth));
		
		constexpr double minv = - 2. * 2.05;
		const double maxz = mfpc.v2obj(minv);
		const double minz = 8. * std::ceil(mfpc.focal());
		const double maxv = mfpc.obj2v(minz);
#if EXPORT_COST_FUNCTION		
		functors.shrink_to_fit();
		std::vector<P3D> xys; xys.reserve(functors.size());
		constexpr double nbsample = 1000.;
		const double stepz = (maxz - minz) / nbsample;
		const double stepv = (maxv - minv) / nbsample;
		
		PRINT_INFO("Evaluate depth from z = "<< minz <<" (v = "<< maxv <<"), to z = " << maxz << " (v = " << minv << ")" << std::endl);
		for(double v = minv; v < maxv; v+=stepv)
		{
			if(std::fabs(v) < 2.) continue;
			
			typename FunctorError_t::ErrorType err;
			VirtualDepth depth{v};
			RMSE cost{0., 0};
			
			for(auto& f : functors)
			{
				if(f(depth, err))
				{
					cost.add(err[0]);	
				}				
			}
			xys.emplace_back(v, mfpc.v2obj(v), cost.get());
		}
		
		PRINT_INFO("Saving data...");
		std::ofstream ofs("costfunction-"+std::to_string(getpid())+"-frame-"+std::to_string(frame)+".csv"); //+"-cluster-"+std::to_string(cluster)+".csv");
		if (!ofs.good())
			throw std::runtime_error(std::string("Cannot open file costfunction.csv"));
		
		ofs << "v,depth,cost\n";
		for(auto& xy: xys) ofs << xy[0] << "," << xy[1]  << "," << xy[2]<< std::endl;
		
		ofs.close();	
#endif	

FORCE_GUI(true);
		PRINT_INFO("Displaying depth map...");
		RawCoarseDepthMap dm{mfpc, minv, maxv};
		for(auto& ob: baps) dm.depth(ob.k, ob.l) = optimized_depth;
		display(dm);
FORCE_GUI(false);
	
	}
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void estimate_depth_from_observations(
	const PlenopticCamera& mfpc,   
	const BAPObservations& observations, /*  (u,v,rho) */
	const std::vector<Image>& images
)
{
//1) Init Parameters
	PRINT_INFO("=== Init Parameter");	
	
	double mdfp = 0.; //mean distance focal plane
	for(int i =0; i < mfpc.I(); ++i) mdfp +=  mfpc.focal_plane(i);
	mdfp /= mfpc.I();
	double v = mfpc.obj2v(mdfp*0.9);
	DEBUG_VAR(mdfp); DEBUG_VAR(v);
	
	std::vector<VirtualDepth> depths(images.size());
	for(int i=0; i<images.size(); ++i) 
		depths[i].v = v; //mfpc.obj2v(2. * mfpc.focal());//8.45; //v; //2.1 ; //mfpc.v2obj(3.);
		
	PRINT_INFO("Initial depth value v = " << depths[0].v <<", z = " << mfpc.v2obj(depths[0].v));
			 
//3) Run optimization
	PRINT_INFO("=== Run optimization");	
	optimize_depths(depths, mfpc, observations, images);
	
	PRINT_INFO("=== Optimization finished!");

	std::getchar();	
}



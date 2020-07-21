#include "depth.h"

//optimization
//#include "optimization/optimization.h"
#include "optimization/depth.h"
#include "optimization/errors/depth.h" //BlurAwareDisparityCostError

#include <pleno/geometry/mia.h> //MicroImage

#include <pleno/processing/improcess.h>

#include <pleno/processing/tools/lens.h>
#include <pleno/processing/tools/rmse.h>

#include <pleno/io/printer.h>
#include <pleno/graphic/display.h>

//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize(
	//OUT
	std::vector<Depth>& depths, /* */
	//IN
	const PlenopticCamera& mfpc,
	const BAPObservations& observations, /*  (u,v,rho) */
	const std::vector<Image>& images
)
{
	constexpr int W = 23u;
	
	using Solver_t = lma::Solver<BlurAwareDisparityCostError>;
	
	//split observations according to frame index
	std::unordered_map<Index /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);	
	
	//for each frame
	for(auto & [frame, baps]: obs)
	{ 		
		if (frame != 3) continue;
		
		PRINT_INFO("Estimation for frame f = " << frame); //<< ", cluster = " << cluster);
		RENDER_DEBUG_2D(
			Viewer::context().layer(Viewer::layer()++)
  				.name("Frame f = "+std::to_string(frame)),
  			images[frame]
	  	);
		
		std::vector<BlurAwareDisparityCostError> functors; functors.reserve(20456);
		
		Solver_t solver{1e2, 50, 1.0 - 1e-12};
		
		Depth depth; depth.z = depths[frame].z;
		
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
					[lhs=*current, &mfpc, &W, img=images[frame], &depth, &solver, &functors](const auto &rhs) -> void {
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
							BlurAwareDisparityCostError{
								viewi, viewj,
								mii, mij,
								mfpc
							},
							&depth
						);
						
						functors.emplace_back(
							BlurAwareDisparityCostError{
								viewi, viewj,
								mii, mij,
								mfpc
							}
						);					
					}
				);
			}//pair of observations
		
		}//clusters

		solver.solve(lma::DENSE, lma::enable_verbose_output());
		PRINT_INFO("Optimized depth for frame ("<< frame<<"), v = " << depth.z << ", z = " << mfpc.v2obj(depth.z));
		
#if 1		
		functors.shrink_to_fit();
		std::vector<P3D> xys; xys.reserve(functors.size());
		
		const double maxz = mfpc.v2obj(2.1);
		const double minv = 2.1;
		const double minz = 4. * std::ceil(mfpc.focal());
		const double maxv = mfpc.obj2v(minz);
		const double nbsample = 1000.;
		const double stepz = (maxz - minz) / nbsample;
		const double stepv = (maxv - minv) / nbsample;
		
		PRINT_INFO("Evaluate depth from z = "<< minz <<" (v = "<< maxv <<"), to z = " << maxz << " (v = " << minv << ")" << std::endl);
		//for(double z = minz; z < maxz; z+=stepz)
		for(double v = minv; v < maxv; v+=stepv)
		{
			//double v = mfpc.obj2v(z);
			BlurAwareDisparityCostError::ErrorType err;
			Depth depthz{v};
			RMSE cost{0., 0};
			
			for(auto& f : functors)
			{
				f(depthz, err);
				cost.add(err[0]);					
			}
			xys.emplace_back(v, mfpc.v2obj(v), cost.get());
		}
		
		std::ofstream ofs("costfunction-"+std::to_string(getpid())+"-frame-"+std::to_string(frame)+".csv"); //+"-cluster-"+std::to_string(cluster)+".csv");
		if (!ofs.good())
			throw std::runtime_error(std::string("Cannot open file costfunction.csv"));
		
		ofs << "v,depth,cost\n";
		for(auto& xy: xys) ofs << xy[0] << "," << xy[1]  << "," << xy[2]<< std::endl;
		
		ofs.close();	
#endif		
	}
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void estimate_depth(
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
	double v = mfpc.obj2v(mdfp);
	DEBUG_VAR(mdfp); DEBUG_VAR(v);
	
	std::vector<Depth> depths(images.size());
	for(int i=0; i<images.size(); ++i) 
		depths[i].z = v; //2.1 ; //mfpc.v2obj(3.);
			 
//3) Run optimization
	PRINT_INFO("=== Run optimization");	
	optimize(depths, mfpc, observations, images);
	
	PRINT_INFO("=== Optimization finished!");

	std::getchar();	
}

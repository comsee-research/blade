#include "depth.h"

//optimization
//#include "optimization/optimization.h"
#include "optimization/depth.h"
#include "optimization/errors/depth.h" //BlurAwareDisparityCostError

#include <pleno/geometry/mia.h> //MicroImage

#include <pleno/processing/improcess.h>
#include <pleno/processing/tools/lens.h>
#include <pleno/io/printer.h>

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
	constexpr int W = 25u;
	
	using Solver_t = lma::Solver<BlurAwareDisparityCostError>;
	
	//split observations according to frame index
	std::unordered_map<Index /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);	
	
	//for each frame
	for(auto & [frame, baps]: obs)
	{ 		
		if (frame != 5) continue;
		
		PRINT_INFO("Estimation for frame f = " << frame); //<< ", cluster = " << cluster);
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
						
						P2D lhsmlkl = mfpc.mi2ml(lhs.k, lhs.l);
						P2D rhsmlkl = mfpc.mi2ml(rhs.k, rhs.l);
						
						// get micro-images
						MicroImage mii{
							static_cast<std::size_t>(lhs.k), static_cast<std::size_t>(lhs.l),
							mfpc.mia().nodeInWorld(lhs.k,lhs.l),
							W/2.,
							lens_type(I, lhsmlkl[0], lhsmlkl[1])
						};
						
						MicroImage mij{
							static_cast<std::size_t>(rhs.k), static_cast<std::size_t>(rhs.l),
							mfpc.mia().nodeInWorld(rhs.k,rhs.l),
							W/2.,
							lens_type(I, rhsmlkl[0], rhsmlkl[1])
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
		
		functors.shrink_to_fit();
		std::vector<P2D> xys; xys.reserve(functors.size());
		
		solver.solve(lma::DENSE, lma::enable_verbose_output());
		PRINT_INFO("Optimized depth for frame ("<< frame<<"), v = " << depth.z << ", z = " << mfpc.v2obj(depth.z) << std::endl);

		for(double z = 300.; z < 1050.; z+=1.)
		//for(double v = 2.; v < 12.; v+=0.1)
		{
			double v = mfpc.obj2v(z);
			BlurAwareDisparityCostError::ErrorType err;
			Depth depthz{v};
			double cost = 0.;
			
			for(auto& f : functors)
			{
				f(depthz, err);
				cost+=err[0];					
			}
			xys.emplace_back(mfpc.v2obj(v), cost);
		}
		
		std::ofstream ofs("costfunction-"+std::to_string(getpid())+"-frame-"+std::to_string(frame)+".csv"); //+"-cluster-"+std::to_string(cluster)+".csv");
		if (!ofs.good())
			throw std::runtime_error(std::string("Cannot open file costfunction.csv"));
		
		ofs << "depth,cost\n";
		for(auto& xy: xys) ofs << xy[0] << "," << xy[1] << std::endl;
		
		ofs.close();	
		
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
	std::vector<Depth> depths(images.size());
	for(int i=0; i<images.size(); ++i) 
		depths[i].z = 2. ; //mfpc.v2obj(3.);
			 
//3) Run optimization
	PRINT_INFO("=== Run optimization");	
	optimize(depths, mfpc, observations, images);
	
	PRINT_INFO("=== Optimization finished!");

	std::getchar();	
}

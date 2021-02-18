#include "compute.h"

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

#include <pleno/graphic/display.h>
#include "../../graphic/display.h"

#include <pleno/processing/imgproc/trim.h>


#include "geometry/depth/RawCoarseDepthMap.h"

#include "strategy.h"
#include "neighbors.h"
#include "initialization.h"
#include "search.h"

#define DEBUG_PROBA						0

static constexpr bool use_true_sigma		= true;
static constexpr bool use_sigmac			= false and use_true_sigma;
static constexpr bool use_sigmav			= (not use_sigmac) and use_true_sigma;
static constexpr bool use_optimization 		= false;

//******************************************************************************
//******************************************************************************
//******************************************************************************
bool is_contrasted_enough(
	const MIA& mia, const Image& scene,
	std::size_t ck, std::size_t cl
) 
{
	constexpr double threshold_contrast = 5.;
	
	const int W 		= int(std::ceil(mia.diameter()));
	const auto center 	= mia.nodeInWorld(ck,cl); 
	const double radius = mia.radius() - mia.border();
	
	Image r;
	cv::getRectSubPix(
		scene, cv::Size{W,W}, 
		cv::Point2d{center[0], center[1]}, r
	);				
	Image m = r.clone();
	trim_binarize(m, radius);
	
	cv::Scalar mean, std;
	cv::meanStdDev(r, mean, std, m); 
	
	const double contrast = std[0];
	
	return (contrast >= threshold_contrast);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap(
	RawCoarseDepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	DepthEstimationStrategy strategies
)
{	
	constexpr double nbsample = 15.;
	constexpr double N = 1.96; //FIXME: increase N ?
	
	std::queue<IndexPair> microimages;
	microimages.emplace(kinit, linit);
	
	while (not (microimages.empty()))
	{
		auto [ck, cl] = microimages.front(); //current indexes
		microimages.pop();
				
		//Already computed, nothing to do
		if (dm.state(ck,cl) == DepthInfo::State::COMPUTED) continue;
		
		//Compute first hypothesis using inner ring
		if (dm.state(ck,cl) == DepthInfo::State::UNINITIALIZED)
		{
			double hypothesis = 0., cost = 0., sigma = -1.;
			VirtualDepth vd{hypothesis};		
							 	
			//get neighbors
			std::map<double, std::vector<IndexPair>> ordered_neighs = neighbors_by_rings(mfpc.mia(), ck, cl, 3.5, 2.);
			
			//filter texture		 	
		 	if (not (strategies.filter) or is_contrasted_enough(mfpc.mia(), scene, ck, cl))
		 	{
			 	initialize_depth(
			 		vd, &cost, &sigma,
			 		ordered_neighs[1.7], mfpc, scene, //micro-images of the same type
			 		ck, cl,
			 		dm.min_depth(), dm.max_depth(), nbsample,
			 		strategies.pairing, strategies.search, false // strategies.metric
			 	);
			 	
			 	hypothesis = vd.v;
		 	}
		 	
		 	if (cost == 0. or not dm.is_valid_depth(hypothesis))
		 	{
		 		dm.depth(ck, cl) = DepthInfo::NO_DEPTH;
		 		dm.state(ck, cl) = DepthInfo::State::COMPUTED;	
		 		
		 		for (auto &n: ordered_neighs[1.]) 
		 		{
		 			if (dm.state(n.k, n.l) == DepthInfo::State::UNINITIALIZED)  
		 			{
		 				microimages.push(n);
		 			}
		 		}
		 		continue;		 	
		 	}
		 	
		 	dm.depth(ck, cl) = hypothesis;
		 	dm.state(ck, cl) = DepthInfo::State::INITIALIZED;				
		}
		
		//Compute depth hypothesis
		if(dm.state(ck,cl) == DepthInfo::State::INITIALIZED)
		{
			double depth = dm.depth(ck, cl), cost = 0., sigma = -1.;
			VirtualDepth vd{depth};		
			
			//compute neighbors
			std::vector<IndexPair> neighs = neighbors(dm.mia(), ck, cl, std::ceil(std::fabs(depth)), 2., 12.);

			//compute hypothesis
			if (strategies.search == SearchStrategy::NONLIN_OPTIM)
			{	
				optimize_depth(vd, &cost, neighs, mfpc, scene, ck, cl, strategies.pairing);
			}
			else if (strategies.search == SearchStrategy::BRUTE_FORCE)
			{
				const double stepv = (dm.max_depth() - dm.min_depth()) / nbsample;
				const double dmin = depth - stepv;
				const double dmax = depth + stepv;
				
				bruteforce_depth(vd, &cost, &sigma, 
					neighs, mfpc, scene, 
					ck, cl, dmin, dmax, nbsample, 
					strategies.pairing, strategies.metric
				);
		 	}
		 	else if (strategies.search == SearchStrategy::GOLDEN_SECTION)
			{
				const double dmin = std::max(depth - N, dm.min_depth());;
				const double dmax = depth + N;
			
				gss_depth(vd, &cost, &sigma, 
					neighs, mfpc, scene, 
					ck, cl, dmin, dmax, std::sqrt(strategies.precision), 
					strategies.pairing, strategies.metric
				);
			}
		 	
			if (cost != 0. and dm.is_valid_depth(vd.v)) 
			{
				depth = vd.v;
				dm.depth(ck, cl) = depth;
			}
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
				if(strategies.belief == BeliefPropagationStrategy::ALL_NEIGHS and dm.is_valid_depth(depth))
				{		
					dm.depth(n.k, n.l) = depth;
			 		dm.state(n.k, n.l) = DepthInfo::State::INITIALIZED;			
				}
				//add to queue
				microimages.push(n);
			}
			
			if(strategies.belief == BeliefPropagationStrategy::FIRST_RING and dm.is_valid_depth(depth))
			{		
				std::vector<IndexPair> innerneighs = inner_ring(dm.mia(), ck, cl);
				
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
void compute_probabilistic_depthmap(
	RawCoarseDepthMap& dm,	RawCoarseDepthMap& confidencedm,
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	DepthEstimationStrategy strategies
)
{	
#if DEBUG_PROBA
	kinit = 144u; linit = 63u;
	//kinit = 133u; linit = 69u;
	//kinit = 30u; linit = 24u;
#endif

	constexpr double nbsample = 15.;
	constexpr double N = 1.96;
	constexpr double PHI = (1. + std::sqrt(5.)) / 2.;
	constexpr double PHI2 = PHI * PHI;
	
	std::queue<IndexPair> microimages;
	microimages.emplace(kinit, linit);
	
	while (not (microimages.empty()))
	{
		auto [ck, cl] = microimages.front(); //current indexes
		microimages.pop();
	
		//Already computed, nothing to do
		if (dm.state(ck, cl) == DepthInfo::State::COMPUTED) continue;
	
		//Compute hypothesis
		if (dm.state(ck, cl) == DepthInfo::State::UNINITIALIZED)
		{
			double hypothesis = 0., cost = 0., sigma = -1.;
			VirtualDepth vd{hypothesis};		
							 	
			//get neighbors		 	
		 	std::map<double, std::vector<IndexPair>> ordered_neighs = neighbors_by_rings(mfpc.mia(), ck, cl, 3.5, 2., 12.);
		 	
		 	for (auto &n: ordered_neighs[1.]) 
	 		{
	 			if (dm.state(n.k, n.l) == DepthInfo::State::UNINITIALIZED)  
	 			{
	 				microimages.push(n);
	 			}
	 		}
		 	
		 	//filter texture		 	
		 	if (not (strategies.filter) or is_contrasted_enough(mfpc.mia(), scene, ck, cl))
		 	{
			 	initialize_depth(
			 		vd, &cost, &sigma,
			 		ordered_neighs[1.7], mfpc, scene, 
			 		ck, cl,
			 		dm.min_depth(), dm.max_depth(), nbsample,
			 		strategies.pairing, strategies.search, false //strategies.metric
			 	);
			 	
			 	hypothesis = vd.v;
		 	}
		 	
		 	if (cost == 0. or not dm.is_valid_depth(hypothesis))
		 	{
		 		dm.depth(ck, cl) = DepthInfo::NO_DEPTH;
		 		dm.state(ck, cl) = DepthInfo::State::COMPUTED;	
		 		confidencedm.depth(ck, cl) = DepthInfo::NO_DEPTH;
		 		
		 		continue;		 	
		 	}
		 	
		 	dm.depth(ck, cl) = hypothesis;
		 	dm.state(ck, cl) = DepthInfo::State::INITIALIZED;
		 	
		 	double muz = 1. / hypothesis;
		 	double sigmaz = 0.;
		 	//update inverse depth observation
	 		if constexpr (use_true_sigma)
	 		{	
		 		const double dmin = dm.min_depth();
				const double dmax = dm.max_depth(); 
		 		double stepv = 0.;
		 		
		 		if (strategies.search == SearchStrategy::BRUTE_FORCE ) stepv = (dmax - dmin) / nbsample;
		 		else if (strategies.search == SearchStrategy::GOLDEN_SECTION ) stepv = sigma;
		 		
		 		const double sigmav = (stepv * stepv);
	 		
	 			sigmaz = muz * muz * muz * muz * sigmav;
	 		}
	 		else 
	 		{
	 			const double B = std::sin(60.) * 2. * mfpc.mia().diameter() / mfpc.sensor().scale();
	 			sigmaz = 1. / (B * B);
	 		}
	 		
	 		confidencedm.depth(ck, cl) = sigmaz ;
	 		 
	 	#if DEBUG_PROBA	
	 		const double dmin = dm.min_depth();
			const double dmax = dm.max_depth(); 
			const double sigmav = hypothesis * hypothesis * std::sqrt(sigmaz);
			
		 	DEBUG_VAR(dmin);
		 	DEBUG_VAR(dmax);
	 		PRINT_DEBUG("**********************");
	 		DEBUG_VAR(hypothesis);
	 		DEBUG_VAR(muz);
	 		DEBUG_VAR(sigmaz);
	 		DEBUG_VAR(sigmav);
	 		
	 		PRINT_INFO("Init at baseline b = " << 1.7 << std::endl);
	 	#endif 		
		}
		
		//Compute depth hypothesis
		if(dm.state(ck,cl) == DepthInfo::State::INITIALIZED)
		{
			double muz = 1. / dm.depth(ck, cl);
			double sigmaz = confidencedm.depth(ck, cl);
			
			const double maxdepth = (1. / muz) + PHI2; //dm.max_depth(); // 
			
			std::map<double, std::vector<IndexPair>> ordered_neighs = 
				neighbors_by_rings(mfpc.mia(), ck, cl, maxdepth, 2., 12.);
						
			for(auto& [baseline, neighs]: ordered_neighs)
			{
		 		const auto& [nk, nl] = neighs[0];
		 		const double B = (mfpc.mla().nodeInWorld(ck, cl) - mfpc.mla().nodeInWorld(nk, nl)).head(2).norm() / mfpc.sensor().scale();
		 		const double M =  neighs.size();
			 	
			 	//optimize depth
			 	double depth = 1. / muz, cost = -1., stddev = -1.;
			 	VirtualDepth vd{depth};	 
			 	
			 	//inverse depth observation
			 	double z = 0.;
			 	double varz = 1. / (B * B);		
			 	
			 	if (strategies.search == SearchStrategy::NONLIN_OPTIM)
				{	
					optimize_depth(vd, &cost, neighs, mfpc, scene, ck, cl, strategies.pairing);
					
					depth = vd.v;
					//compute inverse depth observation
					z = 1. / depth;
				}
				else if (strategies.search == SearchStrategy::BRUTE_FORCE)
				{
					const double dmin = 1. / (muz + N * std::sqrt(sigmaz));
					const double dmax = 1. / (muz - N * std::sqrt(sigmaz)); 
					const double minv = std::max(std::min(dmin, dmax), dm.min_depth());
					const double maxv = std::max(dmax, dmin);
					
					bruteforce_depth(vd, &cost, &stddev, 
						neighs, mfpc, scene, 
						ck, cl, minv, maxv, nbsample, 
						strategies.pairing
					);
					depth = vd.v;	
					
					//compute inverse depth observation
					z = 1. / depth;
					
					if constexpr (use_sigmac)
					{
						//compute derivative
						const double h = 0.05;
						double fcost = 0.;
						VirtualDepth vdph{0.};
						
						bruteforce_depth(vdph, &fcost, nullptr,
							neighs, mfpc, scene,
							ck, cl, vd.v+h, vd.v+2.*h, 1.,
							strategies.pairing
						);
						
						if((fcost - cost) < 1e-7) //no derivative
						{
							varz = 2. * sigmaz;	
						}
						else 
						{
							const double dvdc = h / (fcost - cost);	
	 						const double sigmac = (cost * cost) / (2. * M);
		 					varz = z * z * z * z * dvdc * dvdc * sigmac;	
		 				}		
					}
					else if constexpr (use_sigmav)
					{
						const double stepv = (maxv - minv) / nbsample; 
		 				const double sigmav = (stepv * stepv);
		 				varz = z * z * z * z * sigmav;
					}
						
				#if DEBUG_PROBA
					const double stepv = (maxv - minv) / nbsample; 
				 	DEBUG_VAR(dmin);
				 	DEBUG_VAR(dmax);
				 	DEBUG_VAR(stepv);
			 		PRINT_DEBUG("**********************");
			 		DEBUG_VAR(depth);
				 	DEBUG_VAR(cost);
				 	DEBUG_VAR(stddev);
			 		PRINT_DEBUG("**********************");
			 	#endif 	
				}
				else if (strategies.search == SearchStrategy::GOLDEN_SECTION)
				{
					const double dmin = (1. / muz) - N;
					const double dmax = (1. / muz) + N;
					const double minv = std::max(std::min(dmin, dmax), dm.min_depth());
					const double maxv = std::max(dmax, dmin);
					
					gss_depth(vd, &cost, &stddev, 
						neighs, mfpc, scene, 
						ck, cl, minv, maxv, std::sqrt(strategies.precision), 
						strategies.pairing
					);
					
					depth = vd.v;	
					//compute inverse depth observation
					z = 1. / depth;
					if constexpr (use_true_sigma)
					{
						const double sigmav = (stddev * stddev);
		 				varz = z * z * z * z * sigmav;
					}
					
				#if DEBUG_PROBA
				 	DEBUG_VAR(dmin);
				 	DEBUG_VAR(dmax);
				 	DEBUG_VAR(maxv - minv);
			 		PRINT_DEBUG("**********************");
			 		DEBUG_VAR(depth);
				 	DEBUG_VAR(cost);
				 	DEBUG_VAR(stddev);
			 		PRINT_DEBUG("**********************");
			 	#endif 				
				}			
			 	//if no observation has been taken into account or estimation is not valid, continue
				if (not dm.is_valid_depth(depth) or cost == 0.) 
				{
					continue;
				}
				//update hypothesis
				depth = muz;
				stddev = sigmaz;
								
				muz = (sigmaz * z + varz * muz) / (sigmaz + varz);
				sigmaz = (sigmaz * varz) / (sigmaz + varz);	
					
			#if DEBUG_PROBA
				PRINT_INFO(
					"- update from Z ~ N(" << depth << ", "<< stddev << ") to Z' ~ N(" 
					<< muz << ", " << sigmaz << ")" << std::endl
					<< "i.e., v = " << 1. / depth << " (" << mfpc.v2obj(1. / depth) << ") to v' = "  
					<< 1. / muz << " (" << mfpc.v2obj(1. / muz) << ") " 
					<< "at baseline b = " << baseline << std::endl
				);
		 	#endif			 				 			
			}
			
			const double v = (muz == DepthInfo::NO_DEPTH ? DepthInfo::NO_DEPTH : 1. / muz);
			const double confidence = (sigmaz == DepthInfo::NO_DEPTH ? DepthInfo::NO_DEPTH : (v * v * std::sqrt(sigmaz)));
			
			//set depth
	 		dm.state(ck, cl) = DepthInfo::State::COMPUTED;	
	 		dm.depth(ck, cl) = v;	
	 		
	 		confidencedm.depth(ck, cl) = 1e3 * confidence;		
	 	
		#if DEBUG_PROBA	
	 		PRINT_INFO("Depth estimated at v(" << ck << ", " << cl << ") = " << v << " (z = " << mfpc.v2obj(v)  <<") with confidence = " << confidence << std::endl);
	 		if (ck == kinit and cl == linit) wait();
	 	#endif
		}
	}
	PRINT_DEBUG("Local probabilistic depth estimation finished.");
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap_from_obs(
	RawCoarseDepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
)
{	
//1) Initialize hypothesis
	const std::size_t I = (mfpc.I() == 0) ? 1 : mfpc.I();
	
	double mdfp = 0.; //mean distance focal plane
	for(std::size_t i =0; i < I; ++i) mdfp +=  mfpc.focal_plane(i);
	mdfp /= double(I);
	double v = mfpc.obj2v(mdfp*0.8);
	
	VirtualDepth vd{v};

//2) Optimize depth
	optimize_depth_from_obs(
		vd, nullptr, observations, mfpc, scene
	);
	
//3) Build depth map
	for (const auto& ob :  observations)	
	{
		dm.depth(ob.k, ob.l) = vd.v;
		dm.state(ob.k, ob.l) = DepthInfo::COMPUTED;
	}
}

#include "compute.h"

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

#include <pleno/graphic/display.h>
#include "../../graphic/display.h"

#include <pleno/processing/imgproc/trim.h>

#include "geometry/depth/RawDepthMap.h"

#include "depth.h"
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
//FIXME: move to mia ?
bool is_contrasted_enough(
	const MIA& mia, const Image& scene,
	std::size_t ck, std::size_t cl
) 
{
	constexpr double threshold_contrast = 5.;
	
	const int W 		= int(std::ceil(mia.diameter()));
	const auto center 	= mia.nodeInWorld(ck, cl); 
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
#if 0
	DEBUG_VAR(ck);
	DEBUG_VAR(cl);
	DEBUG_VAR(contrast);
#endif
	return (contrast >= threshold_contrast);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap(
	RawDepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
)
{	
	constexpr double nbsample = 15.;
	constexpr double N = 1.96; //FIXME: increase N ?
	
	int nbrejected = 0;
	int nbinvalided = 0;
	int nbcomputed = 0;
	
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
			//get neighbors
			std::map<double, std::vector<IndexPair>> ordered_neighs = neighbors_by_rings(mfpc.mia(), ck, cl, 3.5, 2.);
			
			DepthHypothesis hypothesis;
				hypothesis.k = ck;
				hypothesis.l = cl;
				hypothesis.min = dm.min_depth();
				hypothesis.max = dm.max_depth();
				hypothesis.precision =  nbsample;
							 			
			//add to queue
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
			 	++nbrejected;
			 	
			 	DepthEstimationStrategy strat = strategies;
			 	strat.metric = false;
			 	
			 	initialize_depth(
			 		hypothesis,
			 		ordered_neighs[1.7], mfpc, scene, //micro-images of the same type
			 		strat
			 	);
		 	}
		 	
		 	if (not (hypothesis.is_valid() and dm.is_valid_depth(hypothesis.depth())))
		 	{
		 		++nbinvalided;
		 		
		 		dm.depth(ck, cl) = DepthInfo::NO_DEPTH;
		 		dm.state(ck, cl) = DepthInfo::State::COMPUTED;	

		 		continue;		 	
		 	}
		 	
		 	dm.depth(ck, cl) 		= hypothesis.depth();
		 	dm.confidence(ck, cl) 	= hypothesis.confidence();
		 	dm.state(ck, cl) 		= DepthInfo::State::INITIALIZED;				
		}
		
		//Compute depth hypothesis
		if(dm.state(ck,cl) == DepthInfo::State::INITIALIZED)
		{
			//compute neighbors
			std::vector<IndexPair> neighs = neighbors(mfpc.mia(), ck, cl, std::ceil(std::fabs(dm.depth(ck, cl))), 2., 12.);
			
			DepthHypothesis hypothesis;
				hypothesis.depth() = dm.depth(ck, cl);
				hypothesis.k = ck;
				hypothesis.l = cl;		
			
			//compute hypothesis
			if (strategies.search == SearchStrategy::NONLIN_OPTIM)
			{	
				optimize_depth(hypothesis, neighs, mfpc, scene, strategies);
			}
			else if (strategies.search == SearchStrategy::BRUTE_FORCE)
			{
				const double stepv = (dm.max_depth() - dm.min_depth()) / nbsample;
				
				hypothesis.min = hypothesis.depth() - stepv;;
				hypothesis.max = hypothesis.depth() + stepv;
				hypothesis.precision = nbsample;
				
				bruteforce_depth(
					hypothesis, 
					neighs, mfpc, scene,
					strategies
				);
		 	}
		 	else if (strategies.search == SearchStrategy::GOLDEN_SECTION)
			{				
				hypothesis.min = std::max(hypothesis.depth() - N, dm.min_depth());
				hypothesis.max = hypothesis.depth() + N;
				hypothesis.precision = std::sqrt(strategies.precision);
			
				gss_depth(
					hypothesis,
					neighs, mfpc, scene,
					strategies
				);
			}
			
			bool isValidDepth = dm.is_valid_depth(hypothesis.depth());
		 	
			if (hypothesis.is_valid() and isValidDepth) 
			{
				dm.depth(ck, cl) = hypothesis.depth();
		 		dm.state(ck, cl) = DepthInfo::State::COMPUTED;
				++nbcomputed;
			}
			else 
			{
				dm.depth(ck, cl) = DepthInfo::NO_DEPTH;
		 		dm.state(ck, cl) = DepthInfo::State::COMPUTED;
				++nbrejected;
				
				continue;
			}	
			
			//depth is valid
			for (auto &n: neighs) 
			{
				//if already computed, do nothing
				if (dm.state(n.k, n.l) == DepthInfo::State::COMPUTED) 
				{
					/* Add strategy to check if estimation is coherent */
					continue;
				}
				
				//if already initialized, average hypotheses
				if (dm.state(n.k, n.l) == DepthInfo::State::INITIALIZED)
				{					
					/* Add strategy to update hypothesis */					
					continue;
				}
				
				//else uninitialized then initalize hypothesis if valid
				if (strategies.belief == BeliefPropagationStrategy::ALL_NEIGHS)
				{		
					dm.depth(n.k, n.l) = hypothesis.depth();
			 		dm.state(n.k, n.l) = DepthInfo::State::INITIALIZED;			
				}
			}
			
			if(strategies.belief == BeliefPropagationStrategy::FIRST_RING)
			{		
				std::vector<IndexPair> innerneighs = inner_ring(mfpc.mia(), ck, cl);
				
				for(auto& n: innerneighs)
				{
					if(dm.state(n.k, n.l) == DepthInfo::State::UNINITIALIZED)
					{
						dm.depth(n.k, n.l) = hypothesis.depth();
			 			dm.state(n.k, n.l) = DepthInfo::State::INITIALIZED;	
			 		}
		 		}		
			}	
		}		
	}
	PRINT_DEBUG("Local depth estimation finished.");
	DEBUG_VAR(nbrejected);
	DEBUG_VAR(nbinvalided);
	DEBUG_VAR(nbcomputed);
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_probabilistic_depthmap(
	RawDepthMap& dm,
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
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
			DepthHypothesis hypothesis;
				hypothesis.k = ck;
				hypothesis.l = cl;
				hypothesis.min = dm.min_depth();
				hypothesis.max = dm.max_depth();
				hypothesis.precision =  nbsample;	
							 	
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
			 		hypothesis,
			 		ordered_neighs[1.7], mfpc, scene, 
			 		strategies
			 	);
		 	}
		 	
		 	if (not (hypothesis.is_valid()) or not (dm.is_valid_depth(hypothesis.depth())))
		 	{
		 		dm.depth(ck, cl) 		= DepthInfo::NO_DEPTH;
		 		dm.state(ck, cl) 		= DepthInfo::State::COMPUTED;	
		 		dm.confidence(ck, cl) 	= DepthInfo::NO_CONFIDENCE;
		 		
		 		continue;		 	
		 	}
		 	
		 	dm.depth(ck, cl) = hypothesis.depth();
		 	dm.state(ck, cl) = DepthInfo::State::INITIALIZED;
		 	
		 			 	
		 	//update inverse depth observation
	 		if constexpr (use_true_sigma)
	 		{	
		 		const double dmin = dm.min_depth();
				const double dmax = dm.max_depth(); 
		 		double stepv = 0.;
		 		
		 		if (strategies.search == SearchStrategy::BRUTE_FORCE) stepv = (dmax - dmin) / nbsample;
		 		else if (strategies.search == SearchStrategy::GOLDEN_SECTION) stepv = hypothesis.sigma;
		 		
		 		const double sigmav = (stepv * stepv);
		 		double muz = hypothesis.invdepth();
	 		
	 			hypothesis.sigma = muz * muz * muz * muz * sigmav;
	 		}
	 		else 
	 		{
	 			const double B = std::sin(60.) * 2. * mfpc.mia().diameter() / mfpc.sensor().scale();
	 			hypothesis.sigma = 1. / (B * B);
	 		}
	 		
	 		dm.confidence(ck, cl) = hypothesis.sigma; //at init contains sigma
	 		 
	 	#if DEBUG_PROBA	
	 		const double dmin = dm.min_depth();
			const double dmax = dm.max_depth(); 
			const double sigmav = hypothesis.depth() * hypothesis.depth() * std::sqrt(hypothesis.sigma);
			
		 	DEBUG_VAR(dmin);
		 	DEBUG_VAR(dmax);
	 		PRINT_DEBUG("**********************");
	 		DEBUG_VAR(hypothesis.depth());
	 		DEBUG_VAR(hypothesis.invdepth());
	 		DEBUG_VAR(hypothesis.sigma);
	 		DEBUG_VAR(sigmav);
	 		
	 		PRINT_INFO("Init at baseline b = " << 1.7 << std::endl);
	 	#endif 		
		}
		
		//Compute depth hypothesis
		if(dm.state(ck,cl) == DepthInfo::State::INITIALIZED)
		{
			DepthHypothesis hypothesis;
				hypothesis.depth() 	= dm.depth(ck, cl);
				hypothesis.sigma	= dm.confidence(ck, cl);
				hypothesis.k 		= ck;
				hypothesis.l 		= cl;
							
			const double maxdepth = hypothesis.depth() + PHI2;
			
			std::map<double, std::vector<IndexPair>> ordered_neighs = 
				neighbors_by_rings(mfpc.mia(), ck, cl, maxdepth, 2., 12.);
						
			for(auto& [baseline, neighs]: ordered_neighs)
			{
		 		const auto& [nk, nl] = neighs[0];
		 		
		 		const double B = (mfpc.mla().nodeInWorld(ck, cl) - mfpc.mla().nodeInWorld(nk, nl)).head(2).norm() / mfpc.sensor().scale();
		 		const double M =  neighs.size();
			 				 	
			 	DepthHypothesis nhypothesis;
					nhypothesis.depth() = hypothesis.depth();
					nhypothesis.sigma	= 1. / (B * B);
					nhypothesis.k 		= ck;
					nhypothesis.l 		= cl;
			 	
			 				 	
			 	if (strategies.search == SearchStrategy::NONLIN_OPTIM)
				{	
					optimize_depth(nhypothesis, neighs, mfpc, scene, strategies);
				}
				else if (strategies.search == SearchStrategy::BRUTE_FORCE)
				{					
					const double dmin = 1. / (hypothesis.invdepth() + N * std::sqrt(hypothesis.sigma));
					const double dmax = 1. / (hypothesis.invdepth() - N * std::sqrt(hypothesis.sigma)); 
										
					nhypothesis.min = std::max(std::min(dmin, dmax), dm.min_depth());
					nhypothesis.max = std::max(dmax, dmin);
					nhypothesis.precision = nbsample;
					
					//compute depth hypothesis
					bruteforce_depth(
						nhypothesis,
						neighs, mfpc, scene, 
						strategies
					);
															
					//compute invz sigma 
					if constexpr (use_sigmac)
					{
						//compute derivative
						const double h = 0.05;
						double fcost = 0.;
						VirtualDepth vdph{0.};
						
						DepthHypothesis temp;
							temp.k = ck;
							temp.l = cl;
							temp.min = nhypothesis.depth() + h;
							temp.max = nhypothesis.depth() + 2. * h;
							temp.precision = 1.;
						
						bruteforce_depth(
							temp,
							neighs, mfpc, scene,
							strategies
						);
						
						if((temp.cost - nhypothesis.cost) < 1e-7) //no derivative
						{
							nhypothesis.sigma = 2. * hypothesis.sigma;	
						}
						else 
						{
							const double dvdc = h / (temp.cost - nhypothesis.cost);	
	 						const double sigmac = (nhypothesis.cost * nhypothesis.cost) / (2. * M);
							const double z = nhypothesis.invdepth();
							
		 					nhypothesis.sigma = z * z * z * z * dvdc * dvdc * sigmac;	
		 				}		
					}
					else if constexpr (use_sigmav)
					{
						const double stepv = (nhypothesis.max - nhypothesis.min) / nbsample; 
		 				const double sigmav = (stepv * stepv);
						const double z = nhypothesis.invdepth();
						
		 				nhypothesis.sigma = z * z * z * z * sigmav;
					}
						
				#if DEBUG_PROBA
					const double stepv = (nhypothesis.max - nhypothesis.min) / nbsample; 
				 	DEBUG_VAR(nhypothesis.min);
				 	DEBUG_VAR(nhypothesis.max);
				 	DEBUG_VAR(stepv);
			 		PRINT_DEBUG("**********************");
			 		DEBUG_VAR(nhypothesis.depth());
				 	DEBUG_VAR(nhypothesis.cost);
				 	DEBUG_VAR(nhypothesis.sigma);
			 		PRINT_DEBUG("**********************");
			 	#endif 	
				}
				else if (strategies.search == SearchStrategy::GOLDEN_SECTION)
				{
					const double dmin = hypothesis.depth() - N;
					const double dmax = hypothesis.depth() + N;
										
					nhypothesis.min = std::max(std::min(dmin, dmax), dm.min_depth());
					nhypothesis.max = std::max(dmax, dmin);
					nhypothesis.precision = std::sqrt(strategies.precision);
					
					gss_depth(
						nhypothesis,
						neighs, mfpc, scene, 
						strategies
					);
												
					if constexpr (use_true_sigma)
					{
						const double sigmav = (nhypothesis.sigma * nhypothesis.sigma);
						const double z = nhypothesis.invdepth();
		 				nhypothesis.sigma = z * z * z * z * sigmav;
					}
					
				#if DEBUG_PROBA
				 	DEBUG_VAR(dmin);
				 	DEBUG_VAR(dmax);
			 		PRINT_DEBUG("**********************");
			 		DEBUG_VAR(nhypothesis.depth());
				 	DEBUG_VAR(nhypothesis.cost);
				 	DEBUG_VAR(nhypothesis.sigma);
			 		PRINT_DEBUG("**********************");
			 	#endif 				
				}
							
			 	//if no observation has been taken into account or estimation is not valid, continue
				if (not (nhypothesis.is_valid()) or not (dm.is_valid_depth(nhypothesis.depth()))) 
				{
					continue;
				}
								
				double z = hypothesis.invdepth();
				double s = hypothesis.sigma;
				
				//update hypothesis
				hypothesis += nhypothesis;
					
			#if DEBUG_PROBA
				PRINT_INFO(
					"- update from Z ~ N(" << z << ", "<< s << ") to Z' ~ N(" 
					<< hypothesis.invdepth() << ", " << hypothesis.sigma << ")" << std::endl
					<< "i.e., v = " << 1. / z << " (" << mfpc.v2obj(1. / z) << ") to v' = "  
					<< hypothesis.depth << " (" << mfpc.v2obj(hypothesis.depth) << ") " 
					<< "at baseline b = " << baseline << std::endl
				);
		 	#endif			 				 			
			}
			
			//const double v = (hypothesis.invdepth() == DepthInfo::NO_DEPTH ? DepthInfo::NO_DEPTH : hypothesis.depth());
			//const double confidence = (hypothesis.sigma == DepthInfo::NO_DEPTH ? DepthInfo::NO_DEPTH : (v * v * std::sqrt(hypothesis.sigma)));
			
			//set depth
	 		dm.state(ck, cl) = DepthInfo::State::COMPUTED;	
	 		dm.depth(ck, cl) = hypothesis.depth();	
	 		
	 		dm.confidence(ck, cl) = hypothesis.confidence(); //1e3 * confidence;		
	 	
		#if DEBUG_PROBA	
	 		PRINT_INFO("Depth estimated at v(" << ck << ", " << cl << ") = " << hypothesis.depth() << " (z = " << mfpc.v2obj(hypothesis.depth())  <<") with confidence = " << confidence << std::endl);
	 		
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
	RawDepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
)
{	
//1) Initialize hypothesis
	const std::size_t I = (mfpc.I() == 0) ? 1 : mfpc.I();
	
	double mdfp = 0.; //mean distance focal plane
	for(std::size_t i =0; i < I; ++i) mdfp +=  mfpc.focal_plane(i);
	mdfp /= double(I);
	const double v = mfpc.obj2v(mdfp*0.8);
	
	DepthHypothesis hypothesis;
		hypothesis.depth() = v;

//2) Optimize depth
	optimize_depth_from_obs(
		hypothesis, observations, mfpc, scene
	);
	
//3) Build depth map
	for (const auto& ob : observations)	
	{
		dm.depth(ob.k, ob.l) = hypothesis.depth();
		dm.state(ob.k, ob.l) = DepthInfo::COMPUTED;
	}
}

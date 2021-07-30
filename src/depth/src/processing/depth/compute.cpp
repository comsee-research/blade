#include "compute.h"

#include <algorithm>

#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

#include <pleno/graphic/display.h>
#include "../../graphic/display.h"

#include <pleno/processing/imgproc/trim.h>

#include "geometry/depth/depthmap.h"

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
	const double radius = mia.radius() - mia.border() - 1.5;
	
	Image mi;
	cv::getRectSubPix(
		scene, cv::Size{W,W}, 
		cv::Point2d{center[0], center[1]}, mi
	);				
	Image mask = mi.clone();
	trim_binarize(mask, radius);
	
	cv::Scalar mean, std;
	cv::meanStdDev(mi, mean, std, mask); 
	
	const double contrast = std[0];
#if 0
	DEBUG_VAR(ck);
	DEBUG_VAR(cl);
	DEBUG_VAR(contrast);
#endif
	return (contrast >= threshold_contrast);
}

bool is_pixel_contrasted_enough(
	const MIA& mia, const Image& scene,
	double u, double v
) 
{
	constexpr double threshold_contrast = 5.;
	constexpr int W = 3;
	
	Image r;
	cv::getRectSubPix(
		scene, cv::Size{W,W}, 
		cv::Point2d{u, v}, r
	);				
	
	cv::Scalar mean, std;
	cv::meanStdDev(r, mean, std); 
	
	const double contrast = std[0];

	return (contrast >= threshold_contrast);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap(
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
)
{	
	constexpr double nbsample = 15.;
	constexpr double N = 1.96; //FIXME: increase N ?
	
	DEBUG_ASSERT((dm.is_coarse_map()), "The map type must be set to COARSE.");
		
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
				hypothesis.precision = nbsample;
							 			
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
			 	DepthEstimationStrategy strat = strategies;
			 	strat.metric = false; //force init in virtual space
			 	
			 	initialize_depth(
			 		hypothesis, ordered_neighs[1.7],  //micro-images of the same type
			 		mfpc, scene, strat
			 	);
		 	}
		 	
		 	if (not (hypothesis.is_valid() and dm.is_valid_depth(hypothesis.depth())))
		 	{		 		
		 		dm.depth(ck, cl) 		= DepthInfo::NO_DEPTH;
		 		dm.confidence(ck, cl) 	= DepthInfo::NO_CONFIDENCE;
		 		dm.state(ck, cl)		= DepthInfo::State::COMPUTED;	 	
		 	}
		 	else
		 	{		 	
		 		dm.depth(ck, cl) 		= hypothesis.depth();
		 		dm.confidence(ck, cl) 	= hypothesis.confidence();
		 		dm.state(ck, cl) 		= DepthInfo::State::INITIALIZED;				
		 	}
		}
		
		//Compute depth hypothesis
		if(dm.state(ck,cl) == DepthInfo::State::INITIALIZED)
		{
			//compute neighbors
			const double maxabsv = std::ceil(std::fabs(dm.depth(ck, cl)));
			std::vector<IndexPair> neighs = neighbors(mfpc.mia(), ck, cl, maxabsv, 2., 12.);
			
			//filter neighbors not having enougth contrast
			neighs.erase(
				std::remove_if(neighs.begin(), neighs.end(),
					[&mfpc, &scene](const IndexPair& n) -> bool {
						return not(is_contrasted_enough(mfpc.mia(), scene, n.k, n.l));
					}
				), 
				neighs.end()
			);
			neighs.shrink_to_fit();
			
		#if 0 //SAME TYPE ONLY	
			neighs.erase(
				std::remove_if(neighs.begin(), neighs.end(),
					[t = mfpc.mia().type(3u, ck, cl), &mfpc](const IndexPair& n) -> bool {
						return (mfpc.mia().type(3u, n.k, n.l) != t);
					}
				), 
				neighs.end()
			);
			neighs.shrink_to_fit();
		#endif	
		
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
				
				hypothesis.min = hypothesis.depth() - stepv;
				hypothesis.max = hypothesis.depth() + stepv;
				hypothesis.precision = nbsample;
				
				bruteforce_depth(
					hypothesis, neighs, mfpc, scene, strategies
				);
		 	}
		 	else if (strategies.search == SearchStrategy::GOLDEN_SECTION)
			{				
				hypothesis.min = std::max(hypothesis.depth() - N, dm.min_depth());
				hypothesis.max = hypothesis.depth() + N;
				hypothesis.precision = std::sqrt(strategies.precision);
			
				gss_depth(
					hypothesis, neighs, mfpc, scene, strategies
				);
			}
			
			bool isValidDepth = dm.is_valid_depth(hypothesis.depth());
		 	
			if (hypothesis.is_valid() and isValidDepth) 
			{
				dm.depth(ck, cl) 		= hypothesis.depth();
				dm.confidence(ck, cl) 	= hypothesis.confidence();
		 		dm.state(ck, cl) 		= DepthInfo::State::COMPUTED;
			}
			else 
			{
				dm.depth(ck, cl) 		= DepthInfo::NO_DEPTH;
				dm.confidence(ck, cl) 	= DepthInfo::NO_CONFIDENCE;
		 		dm.state(ck, cl) 		= DepthInfo::State::COMPUTED;
				
				continue;
			}		
	#if 0		
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
	#endif			
		}		
	}
	PRINT_DEBUG("Local depth estimation finished.");
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_refined_depthmap(
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
)
{		
	constexpr double nbsample = 15.;
	constexpr double N = 1.96; //FIXME: increase N ?
	
	DEBUG_ASSERT((dm.is_refined_map()), "The map type must be set to REFINED.");
	
	std::queue<IndexPair> microimages;
	microimages.emplace(kinit, linit);
	
	while (not (microimages.empty()))
	{
		auto [ck, cl] = microimages.front(); //current indexes
		microimages.pop();
				
		std::vector<IndexPair> pixels = pixels_neighbors(mfpc.mia(), mfpc.sensor().width(), mfpc.sensor().height(), ck, cl);	
		
		auto is_mi_state = [&pixels, &dm](DepthInfo::State state) -> bool {
			for (const auto& [u, v] : pixels)
				if (dm.state(u, v) == state) 
					return true; 
			return false;		
		};
		
		auto min_max_mi_depth_hypothesis = [&pixels, &dm]() -> std::pair<double, double> {
			double min = 1e9;
			double max = -1e9;
			for (const auto& [u, v] : pixels)
			{
				if (dm.state(u,v) != DepthInfo::State::UNINITIALIZED)
				{
					const double d = dm.depth(u, v);
					if (d != DepthInfo::NO_DEPTH) 
					{
						if (d > max) max = d;
						if (d < min) min = d;
					}
				}
			}			
			return {min, max};
		};
			
		//Already computed, nothing to do
		if (is_mi_state(DepthInfo::State::COMPUTED)) continue;
		
		//Compute first hypothesis using inner ring
		if (is_mi_state(DepthInfo::State::UNINITIALIZED))
		{
			//get neighbors
			std::map<double, std::vector<IndexPair>> ordered_neighs = neighbors_by_rings(mfpc.mia(), ck, cl, 3.5, 2.);
			
			//add to queue
			for (auto &n: ordered_neighs[1.]) microimages.push(n);

			//filter at micro-image level	
			const bool mi_rejected = (strategies.filter) and not(is_contrasted_enough(mfpc.mia(), scene, ck, cl));
			
			for (const auto& [u, v] : pixels)
			{
				DepthHypothesis hypothesis;
					//set microimage indexes
					hypothesis.k = ck;
					hypothesis.l = cl;
					//set min/max depth
					hypothesis.min = dm.min_depth();
					hypothesis.max = dm.max_depth();
					//set nb sample to draw
					hypothesis.precision = nbsample;
					//set pixel coordinates
					hypothesis.u = static_cast<double>(u);
					hypothesis.v = static_cast<double>(v);	
				
				const bool pixel_rejected = (strategies.filter) and not(is_pixel_contrasted_enough(mfpc.mia(), scene, u, v));
						 	
			 	if (not mi_rejected and not pixel_rejected) //filter texture
			 	{
				 	DepthEstimationStrategy strat = strategies;
				 	strat.metric = false; //Force init in virtual space
				 	
				 	initialize_depth(
				 		hypothesis,	ordered_neighs[1.7], //micro-images of the same type
				 		mfpc, scene, strat
				 	);
			 	}
			 	
			 	if (not (hypothesis.is_valid() and dm.is_valid_depth(hypothesis.depth())))
			 	{			 		
			 		//PRINT_DEBUG("Invalid depth("<<u<<", "<<v<<") = " << hypothesis.depth());
			 		dm.depth(u, v) 		= DepthInfo::NO_DEPTH;
					dm.confidence(u, v)	= DepthInfo::NO_CONFIDENCE;
			 		dm.state(u, v) 		= DepthInfo::State::COMPUTED;	 	
			 	}
			 	else 
			 	{
				 	//PRINT_DEBUG("depth("<<u<<", "<<v<<") = " << hypothesis.depth());
				 	dm.depth(u, v) 		= hypothesis.depth();
				 	dm.confidence(u, v)	= hypothesis.confidence();
				 	dm.state(u, v) 		= DepthInfo::State::INITIALIZED;
			 	}
			}//end for each pixel				
		}
	
		//Compute depth hypothesis
		if (is_mi_state(DepthInfo::State::INITIALIZED))
		{
			const auto [min, max] = min_max_mi_depth_hypothesis();
			const double maxabsv = std::ceil(std::max(std::fabs(min), std::fabs(max)));
			
			//compute neighbors
			std::vector<IndexPair> neighs = neighbors(mfpc.mia(), ck, cl, maxabsv, 2., 12.);
			
			//filter neighbors not having enougth contrast
			neighs.erase(
				std::remove_if(neighs.begin(), neighs.end(),
					[&mfpc, &scene](const IndexPair& n) -> bool {
						return not(is_contrasted_enough(mfpc.mia(), scene, n.k, n.l));
					}
				), 
				neighs.end()
			);
			neighs.shrink_to_fit();
			
			//for each pixel
			for (const auto& [u, v] : pixels)
			{
				//if already computed pixel
				if (dm.state(u, v) == DepthInfo::COMPUTED) continue;
				
				DepthHypothesis hypothesis;
					hypothesis.depth() = dm.depth(u, v);
					hypothesis.k = ck;
					hypothesis.l = cl;	
					hypothesis.u = static_cast<double>(u);
					hypothesis.v = static_cast<double>(v);	
				
				//compute hypothesis
				if (strategies.search == SearchStrategy::NONLIN_OPTIM)
				{	
					optimize_depth(hypothesis, neighs, mfpc, scene, strategies);
				}
				else if (strategies.search == SearchStrategy::BRUTE_FORCE)
				{
					const double stepv = (dm.max_depth() - dm.min_depth()) / nbsample;
					
					hypothesis.min = min - stepv;
					hypothesis.max = max + stepv;
					hypothesis.precision = nbsample;
					
					bruteforce_depth(
						hypothesis, neighs, mfpc, scene, strategies
					);
			 	}
			 	else if (strategies.search == SearchStrategy::GOLDEN_SECTION)
				{				
					hypothesis.min = std::max(min - N, dm.min_depth());
					hypothesis.max = max + N;
					hypothesis.precision = std::sqrt(strategies.precision);
				
					gss_depth(
						hypothesis, neighs, mfpc, scene, strategies
					);
				}
							 	
				if (hypothesis.is_valid() and dm.is_valid_depth(hypothesis.depth())) 
				{
					dm.depth(u, v) 		= hypothesis.depth();
					dm.confidence(u, v) = hypothesis.confidence();
			 		dm.state(u, v) 		= DepthInfo::State::COMPUTED;
				}
				else 
				{
					dm.depth(u, v) 		= DepthInfo::NO_DEPTH;
					dm.confidence(u, v) = DepthInfo::NO_CONFIDENCE;
			 		dm.state(u, v) 		= DepthInfo::State::COMPUTED;
				}	
			}//end for each pixel			
		}		
	}// end while there is still some mi in queue
	PRINT_DEBUG("Local depth estimation finished.");
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_probabilistic_depthmap(
	DepthMap& dm,
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
	
	DEBUG_ASSERT((dm.is_coarse_map()), "The map type must be set to COARSE.");
	
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
		 		
		 		const double B = (mfpc.mla().nodeInWorld(ck, cl) - mfpc.mla().nodeInWorld(nk, nl)).head<2>().norm() / mfpc.sensor().scale();
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
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
)
{	
	DEBUG_ASSERT((dm.is_coarse_map()), "The map type must be set to COARSE.");
	
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

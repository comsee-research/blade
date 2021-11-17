#include "initialization.h"

#include <random>

#include <pleno/io/printer.h> //DEBUG_ASSERT, PRINT_DEBUG

#include "search.h" //gss, bruteforce

#define USE_SAME_SEED 1
#define COMPUTE_STATS 0


//******************************************************************************
//******************************************************************************
//******************************************************************************
IndexPair initialize_kl(std::size_t ith, std::size_t nbthread, const MIA& mia, InitStrategy mode)
{
#if USE_SAME_SEED 
    static std::mt19937 mt;
#else
    static std::random_device rd;
    static std::mt19937 mt(rd());
#endif
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = mia.width()-1-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = mia.height()-1-margin; 
	const std::size_t lmin = 0+margin;
		
	if (mode == InitStrategy::RANDOM)
	{
		std::uniform_int_distribution<std::size_t> distk(kmin, kmax);
		std::uniform_int_distribution<std::size_t> distl(lmin, lmax);
		
		return {distk(mt), distk(mt)};
	}
	else if (mode == InitStrategy::FROM_LEFT_BORDER)
	{		
		const std::size_t stepl = (lmax - lmin) / (nbthread+1);
		const std::size_t stepk = (kmax - kmin) / 2;
		return {kmin + (ith%2)*stepk , lmin + ith*stepl};	
	}
	else if (mode == InitStrategy::REGULAR_GRID) //FIXME: use kmeans?
	{
		std::vector<IndexPair> v; v.reserve(nbthread);
		
		switch (nbthread)
		{
			case (1): {
				v.emplace_back(kmin+(kmax-kmin)/2, lmin+(lmax-lmin)/2);
				break;
			}
			case (2): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + (lmax-lmin)/2);
				v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + (lmax-lmin)/2);
				break;
			}
			case (3): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
				break;
			}
			case (4): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + 1 * (lmax-lmin)/3);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + 2 * (lmax-lmin)/3);
				v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + 2 * (lmax-lmin)/3);
				break;
			}
			case (5): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
				break;
			}
			case (6): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
				break;
			}
			case (7): {
				v.emplace_back(kmin + 2 * (kmax-kmin)/6 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 4 * (kmax-kmin)/6 , lmin + 1 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/6 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/6 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 5 * (kmax-kmin)/6 , lmin + 2 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 2 * (kmax-kmin)/6 , lmin + 3 * (lmax-lmin)/4);
				v.emplace_back(kmin + 4 * (kmax-kmin)/6 , lmin + 3 * (lmax-lmin)/4);
				break;
			}
			case (8): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
				v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
				v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
				break;
			}
			case (9): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4); 
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
				break;
			}
			case (10): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
				break;
			}
			case (11): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 3 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 3 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 3 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 3 * (lmax-lmin)/4);
				break;
			}
			case (12): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 3 * (lmax-lmin)/4);
				v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 3 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 3 * (lmax-lmin)/4);
				v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 3 * (lmax-lmin)/4);
				break;
			}
			default: {
				ith = 0;
				std::uniform_int_distribution<std::size_t> distk(kmin, kmax);
				std::uniform_int_distribution<std::size_t> distl(lmin, lmax);
			
				v.emplace_back(distk(mt), distk(mt));
				break;
			}
		}
		return v[ith];
	}
	else
	{ 
		return {0,0};
	}
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void initialize_depth(
	DepthHypothesis& hypothesis,
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const DepthEstimationStrategy& strategies
)
{
	constexpr double nbsupplsample = 5.; 
	
#if USE_SAME_SEED 
    static thread_local std::mt19937 mt;
#else
    static thread_local std::random_device rd;
    static thread_local std::mt19937 mt(rd());
#endif
	
	const double stepv = (hypothesis.max - hypothesis.min) / hypothesis.precision;
	const double stepz = (mfpc.v2obj(hypothesis.min) - mfpc.v2obj(hypothesis.max)) / hypothesis.precision;
	
	hypothesis.max += nbsupplsample * stepv; //goes beyond to eliminate wrong hypotheses
	
	//add uniform perturbation on min distance
	if (strategies.randomize)
	{
		std::uniform_real_distribution<double> perturbation(-stepv / 2., stepv / 2.);
		hypothesis.min += perturbation(mt);	
	}
	
	//ensure strategy is not based on optim for init
	SearchStrategy search = strategies.search;
	if (search == SearchStrategy::NONLIN_OPTIM) 
	{
		search = SearchStrategy::GOLDEN_SECTION;
	}
	
	if (search == SearchStrategy::BRUTE_FORCE)
	{
		hypothesis.precision = strategies.metric ? stepz : stepv;
		
		bruteforce_depth(
			hypothesis,
			neighs, mfpc, scene, 
			strategies
		);	
	}
	else if (search == SearchStrategy::GOLDEN_SECTION) 
	{
		hypothesis.precision = strategies.metric ? 1. : std::sqrt(0.1);
				
		gss_depth(
			hypothesis,
			neighs, mfpc, scene, 
			strategies
		);		
	}
	else 
	{
		PRINT_WARN("No initialization implemented for " << search);
	}
	
	
	//PRINT_DEBUG("Initial depth hypothesis at ("<< ck << ", " << cl <<") = " << depth.v);
}

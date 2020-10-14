#include "initialization.h"

#include <random>
#include <variant> //std::variant, std::monostate; std::visit

#include <pleno/processing/tools/rmse.h> //RMSE
#include <pleno/processing/tools/stats.h> //median, mean, iqr, skewness, kurtosis, etc.

#include <pleno/io/printer.h> //DEBUG_ASSERT, PRINT_DEBUG

#include "optimization/depth.h" //lma
#include "optimization/errors/blurawaredisp.h" //BlurAwareDisparityCostError
#include "optimization/errors/disparity.h" //DisparityCostError

#include "depth.h"
#include "pairing.h" //make_functors
#include "search.h" //gss, bruteforce

#define USE_SAME_SEED 1
#define COMPUTE_STATS 0


//******************************************************************************
//******************************************************************************
//******************************************************************************
std::pair<double,double> initialize_min_max_distance(const PlenopticCamera& mfpc)
{
	constexpr double nearfocusd = 1e3;
	constexpr double farfocusd = 5e3; 
	
	const double d = mfpc.distance_focus() * 2.;
	double mind, maxd;
	
	if(d < nearfocusd) //short distances
	{
		maxd = mfpc.distance_focus() * 1.2;
		mind = std::ceil(mfpc.focal()) * 2.; //mfpc.v2obj(12.); //
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
	
	return {mind, maxd};
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
IndexPair initialize_kl(std::size_t i, std::size_t n, const MIA& mia, InitStrategy mode)
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
		const std::size_t stepl = (lmax - lmin) / (n+1);
		const std::size_t stepk = (kmax - kmin) / 2;
		return {kmin + (i%2)*stepk , lmin + i*stepl};	
	}
	else if (mode == InitStrategy::REGULAR_GRID)
	{
		std::vector<IndexPair> v; v.reserve(n);
		
		if (n == 1) v.emplace_back(kmin+(kmax-kmin)/2, lmin+(lmax-lmin)/2);
		else if(n == 2) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + (lmax-lmin)/2);
			v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + (lmax-lmin)/2);
		}
		else if (n == 3) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
			v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
			v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
		}
		else if (n == 4) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + 2 * (lmax-lmin)/3);
			v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + 2 * (lmax-lmin)/3);
		}
		else if (n == 5) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
			v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
			v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
			v.emplace_back(kmin + (kmax-kmin)/2 , lmin + (lmax-lmin)/2);
		}
		else if (n == 6) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 6 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
			v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
			v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
		}
		else if (n == 7) {
			v.emplace_back(kmin + 2 * (kmax-kmin)/6 , lmin + 1 * (lmax-lmin)/4);
			v.emplace_back(kmin + 4 * (kmax-kmin)/6 , lmin + 1 * (lmax-lmin)/4);
			//
			v.emplace_back(kmin + 1 * (kmax-kmin)/6 , lmin + 2 * (lmax-lmin)/4);
			v.emplace_back(kmin + 3 * (kmax-kmin)/6 , lmin + 2 * (lmax-lmin)/4);
			v.emplace_back(kmin + 5 * (kmax-kmin)/6 , lmin + 2 * (lmax-lmin)/4);
			//
			v.emplace_back(kmin + 2 * (kmax-kmin)/6 , lmin + 3 * (lmax-lmin)/4);
			v.emplace_back(kmin + 4 * (kmax-kmin)/6 , lmin + 3 * (lmax-lmin)/4);
		}
		else if (n == 8) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3); 
			v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
			v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
			v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
		}
		else if (n == 9) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4); 
			v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
			v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4); 
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
			v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
			v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
			v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
			v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
		}
		else if (n == 10) {
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
		}
		else if (n == 11) {
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
		}
		else if (n == 12) {
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
		}
		
		return v[i];
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
	VirtualDepth& depth, double* cost, double* sigma, //in/out
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t ck, std::size_t cl,
	double minv, double maxv, double nbsample,
	ObservationsPairingStrategy pairing,
	SearchStrategy search
)
{
	constexpr double nbsupplsample = 5.; 
	
#if USE_SAME_SEED 
    static std::mt19937 mt;
#else
    static std::random_device rd;
    static std::mt19937 mt(rd());
#endif
	
	const double stepv = (maxv - minv) / nbsample;
	
	std::uniform_real_distribution<double> perturbation(-stepv / 2., stepv / 2.);
	maxv = maxv + nbsupplsample * stepv; //goes beyond to eliminate wrong hypotheses
	minv = minv + perturbation(mt);	
	
	if (search == SearchStrategy::BRUTE_FORCE)
	{
		bruteforce_depth(depth, cost, sigma, 
			neighs, mfpc, scene, 
			ck, cl, minv, maxv, nbsample + nbsupplsample, 
			pairing
		);	
	}
	else if (search == SearchStrategy::GOLDEN_SECTION) 
	{
		gss_depth(
			depth, cost, sigma, 
			neighs, mfpc, scene, 
			ck, cl, minv, maxv, std::sqrt(0.1),
			pairing		
		);		
	}
	else 
	{
		PRINT_WARN("No initialization implemented for " << search);
	}
	
	
	//PRINT_DEBUG("Initial depth hypothesis at ("<< ck << ", " << cl <<") = " << depth.v);
}

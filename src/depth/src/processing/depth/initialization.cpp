#include "initialization.h"

#include <random>
#include <variant> //std::variant, std::monostate; std::visit

#include <pleno/processing/tools/rmse.h> //RMSE
#include <pleno/processing/tools/stats.h> //median, mean, iqr, skewness, kurtosis, etc.

#include <pleno/io/printer.h> //DEBUG_ASSERT, PRINT_DEBUG

#include "optimization/depth.h" //lma
#include "optimization/errors/blurawaredisp.h" //BlurAwareDisparityCostError
#include "optimization/errors/disparity.h" //DisparityCostError

#define USE_SAME_SEED 1
#define COMPUTE_STATS 0

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
		
	if(mode == InitStrategy::RANDOM)
	{
		std::uniform_int_distribution<std::size_t> distk(kmin, kmax);
		std::uniform_int_distribution<std::size_t> distl(lmin, lmax);
		
		return {distk(mt), distk(mt)};
	}
	else if (mode == InitStrategy::REGULAR_GRID)
	{
		std::vector<IndexPair> v; v.reserve(n);
		
		if(n == 1) v.emplace_back(kmin+(kmax-kmin)/2, lmin+(lmax-lmin)/2);
		else if(n == 2) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + (lmax-lmin)/2);
			v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + (lmax-lmin)/2);
		}
		else if(n == 3) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
			v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
			v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
		}
		else if(n == 4) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + 2 * (lmax-lmin)/3);
			v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + 2 * (lmax-lmin)/3);
		}
		else if(n == 5) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
			v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
			v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
			v.emplace_back(kmin + (kmax-kmin)/2 , lmin + (lmax-lmin)/2);
		}
		else if(n == 6) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 6 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
			v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
			v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
		}
		else if(n == 7) {
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
		else if(n == 8) {
			v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
			v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3); 
			v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
			v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
			v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
		}
		else if(n == 9) {
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
		else if(n == 10) {
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
		else if(n == 11) {
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
		else if(n == 12) {
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
double initialize_depth(
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t ck, std::size_t cl,
	double minv, double maxv, double nbsample,
	ObservationsParingStrategy mode
)
{
	const std::size_t I = mfpc.I();
	const int W = std::floor(mfpc.mia().diameter());
	
	const bool useBlur = (I > 0u); 
	
	using FunctorsBLADE = std::vector<BlurAwareDisparityCostError>;
	using FunctorsDISP = std::vector<DisparityCostError>;
	using Functors_t = std::variant<FunctorsBLADE, FunctorsDISP>;
	
	Functors_t vfunctor;
		if(useBlur) vfunctor.emplace<FunctorsBLADE>(FunctorsBLADE{});
		else vfunctor.emplace<FunctorsDISP>(FunctorsDISP{});  
	
	double hypothesis = 0.;
		
	std::visit([&](auto&& functors) { 
		using T = std::decay_t<decltype(functors)>;
		using FunctorError_t = typename T::value_type;
				
	 	//compute ref observation
	 	functors.reserve(neighs.size());
	 	
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
				
				functors.emplace_back(
					//FunctorError_t{
						refview, targetview,
						ref, target,
						mfpc
					//}
				);			 	
		 	}
		}
		else
		{
			DEBUG_ASSERT(false, "Can't initialize depth, no other strategy implemented yet");
		}
		
	 	//evaluate observations, find min cost
		const double stepv = (maxv - minv) / nbsample;
		maxv = maxv + 5.*stepv; //goes beyond to eliminate wrong hypotheses
		
		double mincost = 1e9;
		
	#if COMPUTE_STATS		
		double maxcost = 0.;
		std::vector<double> costs; costs.reserve(nbsample);
	#endif
		
		for(double v = minv; v <= maxv; v+=stepv)
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
			
			const double c = cost.get();
			
			if(c < mincost) 
			{
				mincost = c;
				hypothesis = v;
			}
			
		#if COMPUTE_STATS
			if(c > maxcost) maxcost = c;
			costs.emplace_back(c);
		#endif
		}

	#if COMPUTE_STATS	
		double q1, med, q3, iqr_, kurt, skew, mean_, std;
		iqr_ = iqr(costs, q1, med, q3);
		kurt = kurtosis(costs);
		skew = skewness(costs);
		mean_ = mean(costs);
		std = stddev(costs);
		
		std::ostringstream oss;
		oss <<	"Initial depth hypothesis at ("<< ck << ", " << cl <<") = " << hypothesis << std::endl
			<< "\t(min= " << mincost * 100. 
			<< ", max= " << maxcost * 100. 
			<< ", mean=" << mean_ * 100. 
			<< ", std= " << std * 100000. 
			<< ", dist= "<< (mean_ - mincost) / mincost * 100. << std::endl
			<< "\t, med= "<< med * 100. 
			<< ", q1= " << q1 * 100.
			<< ", q3= " << q3 * 100.
			<< ", iqr= " << iqr_ * 100. << std::endl
			<< "\t, skew= " << skew * 100.
			<< ", kurt= " << kurt * 100.
			<< ")";
		
		PRINT_DEBUG(oss.str());
	#else
		//PRINT_DEBUG("Initial depth hypothesis at ("<< ck << ", " << cl <<") = " << hypothesis);
	#endif
	}, vfunctor);

	return hypothesis;
}

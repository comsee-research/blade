#include "search.h"

#include <variant> //std::variant

//optimization
#include "optimization/depth.h"
#include "optimization/errors/blurawaredisp.h" //BlurAwareDisparityCostError
#include "optimization/errors/disparity.h" //DisparityCostError

#include <pleno/processing/tools/rmse.h>
#include <pleno/processing/tools/stats.h>

#include <pleno/io/printer.h>

#include "strategy.h"
#include "neighbors.h"
#include "pairing.h"

#define DEBUG_PROBA						0
#define VERBOSE_OPTIM					0

constexpr double 		AUTOMATIC_LAMBDA_SCALE 	= -1.;

//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize_depth(
	VirtualDepth& depth, double* cost, //in/out
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t ck, std::size_t cl, //current indexes
	ObservationsPairingStrategy pairing
)
{ 	
	const std::size_t I = mfpc.I();	
	const bool useBlur = (I > 0u); 
	
	using FunctorsBLADE = std::vector<BlurAwareDisparityCostError>;
	using FunctorsDISP = std::vector<DisparityCostError>;
	using Functors_t = std::variant<FunctorsBLADE, FunctorsDISP>;
	
	Functors_t vfunctor;
		if(useBlur) vfunctor.emplace<FunctorsBLADE>(FunctorsBLADE{});
		else vfunctor.emplace<FunctorsDISP>(FunctorsDISP{});  
		
	//init virtual depth
	VirtualDepth hypothesis = depth;
	
	std::visit([&](auto&& functors) { 
	//--------------------------------------------------------------------------
		using T = std::decay_t<decltype(functors)>;
		using FunctorError_t = typename T::value_type;
		
		//Create observations
		make_functors(functors, neighs, ck, cl, mfpc, scene, pairing);
		//Create solver
		lma::Solver<FunctorError_t> solver{AUTOMATIC_LAMBDA_SCALE, 50, 1.0 - 1e-12};
		//Add observations to solver
		for (auto&& f: functors) solver.add(f, &hypothesis);
		//Solve
	#if VERBOSE_OPTIM
	 	PRINT_DEBUG("*** Initial depth value v = " << hypothesis.v);
	 	solver.solve(lma::DENSE, lma::enable_verbose_output());
	 	PRINT_DEBUG("*** Optimized depth value v = " << hypothesis.v);
	#else
	 	solver.solve(lma::DENSE); //no verbose, lma::enable_verbose_output());
	#endif
		//Save results
		if(cost) *cost = solver.final_cost;
 		depth.v = hypothesis.v;
	//--------------------------------------------------------------------------
	}, vfunctor);
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize_depth_from_obs(
	VirtualDepth& depth, double* cost, //in/out
	//--------------------------------------------------------------------------
	const BAPObservations& observations, /* (u,v,rho) */
	const PlenopticCamera& mfpc, const Image& scene
)
{
	const std::size_t I = mfpc.I();	
	const bool useBlur = (I > 0u); 
	
	using FunctorsBLADE = std::vector<BlurAwareDisparityCostError>;
	using FunctorsDISP = std::vector<DisparityCostError>;
	using Functors_t = std::variant<FunctorsBLADE, FunctorsDISP>;
	
	Functors_t vfunctor;
		if(useBlur) vfunctor.emplace<FunctorsBLADE>(FunctorsBLADE{});
		else vfunctor.emplace<FunctorsDISP>(FunctorsDISP{});  
		
	//init virtual depth
	VirtualDepth hypothesis = depth;
	
	std::visit([&](auto&& functors) { 
	//--------------------------------------------------------------------------
		using T = std::decay_t<decltype(functors)>;
		using FunctorError_t = typename T::value_type;
		
		//Create observations
		make_functors_from_obs(functors, mfpc, scene, observations);
		
		//Create solver
		lma::Solver<FunctorError_t> solver{AUTOMATIC_LAMBDA_SCALE, 50, 1.0 - 1e-12};
		
		//Add observations to solver
		for (auto&& f: functors) solver.add(f, &hypothesis);
		
		//Solve
	 	PRINT_DEBUG("*** Initial depth value v = " << hypothesis.v);
	 	solver.solve(lma::DENSE, lma::enable_verbose_output());
	 	PRINT_DEBUG("*** Optimized depth value v = " << hypothesis.v);
	
		//Save results
		if(cost) *cost = solver.final_cost;
 		depth.v = hypothesis.v;
	//--------------------------------------------------------------------------
	}, vfunctor);
}



//******************************************************************************
//******************************************************************************
//******************************************************************************
void bruteforce_depth(
	VirtualDepth& depth, double* cost, double* sigma, //in/out
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t ck, std::size_t cl, //current indexes
	double minv, double maxv, double nbsample,
	ObservationsPairingStrategy pairing
)
{
	const std::size_t I = mfpc.I();	
	const bool useBlur = (I > 0u); 
	
	using FunctorsBLADE = std::vector<BlurAwareDisparityCostError>;
	using FunctorsDISP = std::vector<DisparityCostError>;
	using Functors_t = std::variant<FunctorsBLADE, FunctorsDISP>;
	
	Functors_t vfunctor;
		if (useBlur) vfunctor.emplace<FunctorsBLADE>(FunctorsBLADE{});
		else vfunctor.emplace<FunctorsDISP>(FunctorsDISP{});  
	
	std::visit([&](auto&& functors) { 
	//--------------------------------------------------------------------------
		using T = std::decay_t<decltype(functors)>;
		using FunctorError_t = typename T::value_type;
		
		make_functors(functors, neighs, ck, cl, mfpc, scene, pairing);
		
	 	//evaluate observations, find min cost
		const double stepv = (maxv - minv) / nbsample;
		double mincost = 1e9;
		
		if (cost) *cost = 0.;
		if (sigma) *sigma = 0.;
		
		for (double v = minv; v <= maxv; v += stepv)
		{
			if (std::fabs(v) < 2.) continue;
			
			typename FunctorError_t::ErrorType err;
			VirtualDepth hypothesis{v};
			RMSE rmse{0., 0};
			
			std::vector<double> costs; 
			if (sigma) costs.reserve(functors.size());
			
			for (auto& f : functors)
			{
				if (f(hypothesis, err))
				{
					rmse.add(err[0]);
					if (sigma) costs.emplace_back(err[0]*err[0]);	
				}				
			}
			
			const double c = rmse.get();
			
			if (c < mincost) 
			{
				mincost = c;
				
				if (cost) *cost = mincost;
				if (sigma) *sigma = stddev(costs);
				depth.v = hypothesis.v;
			}
		}
	//--------------------------------------------------------------------------
	}, vfunctor);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void gss_depth(
	VirtualDepth& depth, double* cost, double* sigma, //in/out
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t ck, std::size_t cl, //current indexes
	double minv, double maxv, double tol,
	ObservationsPairingStrategy pairing
)
{
	constexpr double invphi 	= (std::sqrt(5.) - 1.) / 2.;
	constexpr double invphi2 	= (3. - std::sqrt(5.)) / 2.;
	
	const std::size_t I = mfpc.I();	
	const bool useBlur = (I > 0u); 
	
	using FunctorsBLADE = std::vector<BlurAwareDisparityCostError>;
	using FunctorsDISP = std::vector<DisparityCostError>;
	using Functors_t = std::variant<FunctorsBLADE, FunctorsDISP>;
	
	Functors_t vfunctor;
		if (useBlur) vfunctor.emplace<FunctorsBLADE>(FunctorsBLADE{});
		else vfunctor.emplace<FunctorsDISP>(FunctorsDISP{});  
	
	std::visit([&](auto&& functors) { 
	//--------------------------------------------------------------------------
		using T = std::decay_t<decltype(functors)>;
		using FunctorError_t = typename T::value_type;
		
		make_functors(functors, neighs, ck, cl, mfpc, scene, pairing);
		
		auto F = [&](double v) -> double {
			typename FunctorError_t::ErrorType err;
			VirtualDepth hypothesis{v};
			//RMSE rmse{0., 0};
			double total_cost = 0.; double total_weight = 0.;
			
			for (auto& f : functors)
			{
				if (f(hypothesis, err))
				{
					const double error 				= err[0];
					const double weight 			= f.weight(v);
					const double weighted_sqr_err	= weight * error * error;
					
					//rmse.add(error);
					total_cost 		+= weighted_sqr_err;
					total_weight  	+= weight;
				}				
			}
			double e = 0.;		
			if (total_weight != 0.) e = std::sqrt(total_cost / total_weight);	
			return e; //rmse.get();
		};
		
	 	//run golden section search to find min cost	 	
	 	double a = std::min(minv, maxv);
	 	double b = std::max(minv, maxv);
	 	double h = b - a;
	 	
	 	const int n = static_cast<int>(std::ceil(std::log(tol / h) / std::log(invphi)));
	 	
		if (cost) *cost = 0.;
		if (sigma) *sigma = (b - a) / 2.;
		depth.v = (a + b) / 2.;
	 	
	 	if (h <= tol) return; 
	 	
		double c = a + invphi2 * h;
		if (c > -2. and c < 2.) c = -2.01;
		double d = a + invphi * h;
		if (d > -2. and d < 2.) d = 2.01;
		
		double yc = F(c);
		double yd = F(d);
		
		if (yc == 0. and yd == 0.) return;
		
		for (int k = 0; k < n; ++k)
		{
		#if DEBUG_PROBA
			PRINT_DEBUG("it("<<k+1<<" / "<<n<<")...");
			DEBUG_VAR(a); DEBUG_VAR(b); DEBUG_VAR(c); DEBUG_VAR(d); DEBUG_VAR(h);
			DEBUG_VAR(yc); DEBUG_VAR(yd);
		#endif				
			if (yc > yd or yc == 0.)
			{
				a = c;
				c = d;
				yc = yd;
				h = invphi * h;
				d = a + invphi * h;
				if (d > -2. and d < 2.) d = 2.01;
				yd = F(d);						
			}
			else //(yc < yd)
			{
				b = d;
				d = c;
				yd = yc;
				h = invphi * h;
				c = a + invphi2 * h;
				if (c > -2. and c < 2.) c = -2.01;
				yc = F(c);
			}	
		}
		
		//if both equals 0 then no estimation
		if (yc == 0. and yd == 0.) return;
							
		if (yc > yd or yc == 0.)
		{
			depth.v = (c + b) / 2.;
			if (cost) *cost = yd;
			if (sigma) *sigma = (b - c) / 2.;
		}
		else
		{
			depth.v = (a + d) / 2.;
			if (cost) *cost = yc;
			if (sigma) *sigma = (d - a) / 2.;
		}	
	//--------------------------------------------------------------------------
	}, vfunctor);
}


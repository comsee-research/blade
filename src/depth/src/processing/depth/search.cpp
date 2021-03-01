#include "search.h"

#include <variant> //std::variant

//optimization
#include "optimization/depth.h"
#include "optimization/errors/disparity.h" //DisparityCostError, BlurAwareDisparityCostError

#include <pleno/processing/tools/stats.h>

#include <pleno/io/printer.h>

#include "pairing.h"

static constexpr double AUTOMATIC_LAMBDA_SCALE 	= -1.;
static constexpr bool 	VERBOSE_OPTIM			= false;

//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize_depth(
	DepthHypothesis& hypothesis,
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const DepthEstimationStrategy& strategies
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
			
	std::visit([&](auto&& functors) { 
	//--------------------------------------------------------------------------
		using T = std::decay_t<decltype(functors)>;
		using FunctorError_t = typename T::value_type;
		
		//Create observations
		make_functors(
			functors, neighs, 
			hypothesis.k, hypothesis.l, 
			mfpc, scene, strategies.pairing,
			hypothesis.u, hypothesis.v
		);
		//Create solver
		lma::Solver<FunctorError_t> solver{AUTOMATIC_LAMBDA_SCALE, 50, 1.0 - 1e-12};
		//Add observations to solver
		for (auto&& f: functors) solver.add(f, &(hypothesis.vd));
		//Solve
		if constexpr (VERBOSE_OPTIM)
	 	{
	 		PRINT_DEBUG("*** Initial depth value v = " << hypothesis.depth());
	 		solver.solve(lma::DENSE, lma::enable_verbose_output());
	 		PRINT_DEBUG("*** Optimized depth value v = " << hypothesis.depth());
		} 
		else
		{
	 		solver.solve(lma::DENSE); //no verbose, lma::enable_verbose_output());
		}
		//Save results
		hypothesis.cost = solver.final_cost;
	//--------------------------------------------------------------------------
	}, vfunctor);
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize_depth_from_obs(
	DepthHypothesis& hypothesis,
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
	
	std::visit([&](auto&& functors) { 
	//--------------------------------------------------------------------------
		using T = std::decay_t<decltype(functors)>;
		using FunctorError_t = typename T::value_type;
		
		//Create observations
		make_functors_from_obs(functors, mfpc, scene, observations);
		
		//Create solver
		lma::Solver<FunctorError_t> solver{AUTOMATIC_LAMBDA_SCALE, 50, 1.0 - 1e-12};
		
		//Add observations to solver
		for (auto&& f: functors) solver.add(f, &(hypothesis.vd));
		
		//Solve
	 	PRINT_DEBUG("*** Initial depth value v = " << hypothesis.depth());
	 	solver.solve(lma::DENSE, lma::enable_verbose_output());
	 	PRINT_DEBUG("*** Optimized depth value v = " << hypothesis.depth());
	
		//Save results
		hypothesis.cost = solver.final_cost;
	//--------------------------------------------------------------------------
	}, vfunctor);
}



//******************************************************************************
//******************************************************************************
//******************************************************************************
void bruteforce_depth(
	DepthHypothesis& hypothesis,
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const DepthEstimationStrategy& strategies
)
{
	const bool metric = strategies.metric;
	const double min = metric ? mfpc.v2obj(hypothesis.max) : hypothesis.min;
	const double max = metric ? mfpc.v2obj(hypothesis.min) : hypothesis.max;
	
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
		
		make_functors(
			functors, neighs, 
			hypothesis.k, hypothesis.l, 
			mfpc, scene, strategies.pairing,
			hypothesis.u, hypothesis.v
		);
		
	 	//evaluate observations, find min cost
		const double step = hypothesis.precision;
		double mincost = 1e9;
				
		for (double d = min; d <= max; d += step)
		{
			const double v = metric ? mfpc.obj2v(d) : d;
			
			if (std::fabs(v) < 2.) continue;
			
			typename FunctorError_t::ErrorType err;
			VirtualDepth depth{v};
			
			std::vector<double> costs; 
			costs.reserve(functors.size());
			
			double total_cost = 0.; double total_weight = 0.;
			
			for (auto& f : functors)
			{
				if (f(depth, err))
				{
					const double error 			= err[0];
					const double weight 		= f.weight(v);
					const double weighted_err 	= weight * error;
					
					total_cost 		+= weighted_err;
					total_weight  	+= weight;
					
					costs.emplace_back(weighted_err);
				}				
			}
						
			const double c = (total_weight != 0.) ? (total_cost / total_weight) : 1e9; //0.;
			
			if (c < mincost) 
			{
				mincost = c;
				
				hypothesis.depth() 	= depth.v;
				hypothesis.cost 	= mincost;
				hypothesis.sigma 	= stddev(costs);
			}
		}
	//--------------------------------------------------------------------------
	}, vfunctor);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void gss_depth(
	DepthHypothesis& hypothesis,
	//--------------------------------------------------------------------------
	const std::vector<IndexPair>& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const DepthEstimationStrategy& strategies
)
{
	constexpr double invphi 	= (std::sqrt(5.) - 1.) / 2.;
	constexpr double invphi2 	= (3. - std::sqrt(5.)) / 2.;

	const bool metric = strategies.metric;
		
	const double min = metric ? mfpc.v2obj(hypothesis.max) : hypothesis.min;
	const double max = metric ? mfpc.v2obj(hypothesis.min) : hypothesis.max;
	
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
		
		make_functors(
			functors, neighs, 
			hypothesis.k, hypothesis.l, 
			mfpc, scene, strategies.pairing, 
			hypothesis.u, hypothesis.v
		);
		
		auto F = [&functors](double v) -> double {
			typename FunctorError_t::ErrorType err;
			
			VirtualDepth depth{v};
			double total_cost = 0.; double total_weight = 0.;
			
			for (auto& f : functors)
			{
				if (f(depth, err))
				{
					const double error 				= err[0];
					const double weight 			= f.weight(v);
					const double weighted_err 		= weight * error;
					
					total_cost 		+= weighted_err;
					total_weight  	+= weight;
				}				
			}
			
			return (total_weight != 0.) ? (total_cost / total_weight) : 0.;
		};
		
	 	//run golden section search to find min cost	 	
	 	double a = std::min(min, max);
	 	double b = std::max(min, max);
	 	double h = b - a;
	 	
	 	const int n = static_cast<int>(std::ceil(std::log(hypothesis.precision / h) / std::log(invphi)));
	 	
		hypothesis.cost = 0.;
		hypothesis.sigma = (b - a) / 2.;
		hypothesis.depth() = metric ? mfpc.obj2v((a + b) / 2.) : (a + b) / 2.;
	 	
	 	if (h <= hypothesis.precision) return; 
	 	
		double c = a + invphi2 * h;
		double c_ = metric ? mfpc.obj2v(c) : c;
		if (std::fabs(c_) < 2.) 
		{
			c_ = -2.01;
			c  = metric ? mfpc.v2obj(c_) : c_;
		}
		
		double d = a + invphi * h;
		double d_ = metric ? mfpc.obj2v(d) : d;
		if (std::fabs(d_) < 2.) 
		{
			d_ = 2.01;
			d  = metric ? mfpc.v2obj(d_) : d_;
		}
		
		double yc = F(c_);
		double yd = F(d_);
		
		if (yc == 0. and yd == 0.) return;
		
		for (int k = 0; k < n; ++k)
		{			
			if (yc > yd or yc == 0.)
			{
				a = c;
				c = d;
				yc = yd;
				h = invphi * h;
				d = a + invphi * h;
				d_ = metric ? mfpc.obj2v(d): d;
				if (std::fabs(d_) < 2.) 
				{
					d_ = 2.01;
					d  = metric ? mfpc.v2obj(d_) : d_;
				}
				yd = F(d_);						
			}
			else //(yc < yd)
			{
				b = d;
				d = c;
				yd = yc;
				h = invphi * h;
				c = a + invphi2 * h;
				c_ = metric ? mfpc.obj2v(c): c;
				if (std::fabs(c_) < 2.) 
				{
					c_ = -2.01;
					c  = metric ? mfpc.v2obj(c_) : c_;
				}
				yc = F(c_);
			}	
		}
		
		//if both equals 0 then no estimation
		if (yc == 0. and yd == 0.) return;
							
		if (yc > yd or yc == 0.)
		{
			hypothesis.depth() = metric ? mfpc.obj2v((c + b) / 2.) : (c + b) / 2.;
			hypothesis.cost = yd;
			hypothesis.sigma = (b - c) / 2.;
		}
		else
		{
			hypothesis.depth() = metric ? mfpc.obj2v((a + b) / 2.) : (a + b) / 2.;
			hypothesis.cost = yc;
			hypothesis.sigma = (d - a) / 2.;
		}	
	//--------------------------------------------------------------------------
	}, vfunctor);
}

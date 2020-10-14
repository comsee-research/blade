#include "export.h"

#include <variant> //std::variant, std::monostate; std::visit

#include <pleno/processing/tools/rmse.h> //RMSE
#include <pleno/processing/tools/stats.h> //median, mean, iqr, skewness, kurtosis, etc.

#include <pleno/io/printer.h> //DEBUG_ASSERT, PRINT_DEBUG
#include <pleno/graphic/gui.h>
#include <pleno/graphic/viewer_2d.h>

#include "optimization/depth.h" //lma
#include "optimization/errors/blurawaredisp.h" //BlurAwareDisparityCostError
#include "optimization/errors/disparity.h" //DisparityCostError

#include "../../types.h"
#include "neighbors.h"
#include "pairing.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
IndexPair extract_micro_image_indexes(
	const Image& gray, const MIA& grid
)
{
	PRINT_WARN("Extracting coordinates of micro-image...");
FORCE_GUI(true);
	
	const double ratio = double(gray.rows) / double(gray.cols);
	const int base_size = 800;
	GUI(
		RENDER_DEBUG_2D(
			Viewer::context().size(base_size,base_size*ratio).layer(Viewer::layer()).name("Image"), 
			gray
		);
    );
    Viewer::update();	
	volatile bool finished = false;
	IndexPair indexes;
	
	Viewer::context().on_click([&](float x, float y)
    {
		const int k = static_cast<int>( x * grid.width() /  gray.cols );
		const int l = static_cast<int>( y * grid.height() / gray.rows );

		auto is_out_of_range = [&](int k, int l) {
			return (k < 0 or k > int(grid.width()) or l < 0  or l > int(grid.height()));
		};
		
		if(is_out_of_range(k,l))
		{
			PRINT_ERR("Click out of bound, unvalidated.");
			return;
		}
		
		GUI(
			auto c = P2D{x,y};
  			Viewer::context().layer(Viewer::layer())
  				.pen_color(v::purple).pen_width(5)
  				.add_circle(c[0], c[1], 12.5)
  				.update();
		);
		
		indexes.k = k; indexes.l = l;
		
		finished = true;
    });
	
	while(not finished);

	Viewer::context().on_click([](float,float){});	
	Viewer::update();

FORCE_GUI(false);
	return indexes;
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_costs(auto& data, auto& functors, const PlenopticCamera& mfpc, double minv, double maxv, double stepv)
{	
	using Functors_t = std::decay_t<decltype(functors)>;
	using FunctorError_t = typename Functors_t::value_type;		
	using Error_t = typename FunctorError_t::ErrorType;
	
	using Data_t = std::decay_t<decltype(data)>;
	using Datum_t = typename Data_t::value_type;
	
	data.reserve(functors.size());
	
	for (double v = minv; v <= maxv; v+=stepv)
	{
		if (std::fabs(v) < 2.) continue;
		
		Error_t err;
		VirtualDepth depth{v};
		
		RMSE cost{0., 0};
		
		std::vector<double> costs; costs.reserve(functors.size());
		std::vector<double> weights; weights.reserve(functors.size());
		
		double total_cost = 0.; double total_weight = 0.; double N = 0.;
		double mu_err = 0., sigma_err = 0.; bool isGaussianInit = false;
		
		for (auto& f : functors)
		{
			if (f(depth, err))
			{
				const double error 				= err[0];
				const double weight 			= f.weight(v);
				const double weighted_err		= weight * error;
				const double sqr_err 			= error * error;
				const double weighted_sqr_err 	= sqr_err * weight;
				const double var				= 1. / weight;
				
				cost.add(error); //for RMSE
				
				costs.emplace_back(error); //for STATS
				weights.emplace_back(weight); //for STATS
				
				total_cost 		+= weighted_sqr_err;//weighted_err;
				total_weight  	+= weight;
				N 				+= 1.;
				
				if(not isGaussianInit)
				{
					mu_err =  error;
					sigma_err = var;
					
					isGaussianInit = true;
					continue;
				}
				
				//update mu and sigma
				mu_err = (sigma_err * sqr_err + var * mu_err) / (sigma_err + var);
				sigma_err = (sigma_err * var) / (sigma_err + var);
			}				
		}
		
		const double mu = mean(costs);
		const double med = median(costs);
		const double sigma = stddev(costs); 
		
		Datum_t d; 
		d <<//v, z, depth
				v, 1. / v, mfpc.v2obj(v), 
			//rmse, cost, nobs
				cost.get(), cost.sum(), N,
			//median, mean, sigma
				med, mu, sigma, 
			//normalized weighed cost, mean weighed cost, wrmse, weight
				total_cost / total_weight, total_cost / N, std::sqrt(total_cost / total_weight), total_weight, 
			//gaussian mean, gaussian sigma, scaled sigma
				mu_err, sigma_err, sigma_err / N; 
			
		data.emplace_back(d);
	}
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void export_cost_function(
	const PlenopticCamera& mfpc, const Image& scene, 
	double minv, double maxv, double nbsample,
	ObservationsPairingStrategy mode,
	bool export_per_baseline
)
{
	const std::size_t I = mfpc.I();
	const int W = std::ceil(mfpc.mia().diameter()); DEBUG_VAR(W);
	
	const bool useBlur = (I > 0u); 
	
	using FunctorsBLADE = std::vector<BlurAwareDisparityCostError>;
	using FunctorsDISP = std::vector<DisparityCostError>;
	using Functors_t = std::variant<FunctorsBLADE, FunctorsDISP>;
	
	Functors_t vfunctor;
		if(useBlur) vfunctor.emplace<FunctorsBLADE>(FunctorsBLADE{});
		else vfunctor.emplace<FunctorsDISP>(FunctorsDISP{});  
	
	const auto [ck, cl] = extract_micro_image_indexes(scene, mfpc.mia());
		
	std::visit([&](auto&& functors) { 
		using T = std::decay_t<decltype(functors)>;
		using FunctorError_t = typename T::value_type;

//------------------------------------------------------------------------------
		std::ofstream ofscf(
			"costfunction-" + std::to_string(getpid())
			+ "-at-" + std::to_string(ck) + "-" + std::to_string(cl) + ".csv"
		); 
		if (not ofscf.good()) throw std::runtime_error(std::string("Cannot open file costfunction.csv"));
		
		std::ostringstream headercsv;
		headercsv << "v,z,depth,baseline,"
				<< "rmse,cost,nbobs,"
				<< "median,mean,sigma,"
				<< "nwcost,mwcost,wrmse,weight,"
				<< "muerr,sigmaerr,ssigmaerr\n";
		
		ofscf << headercsv.str();
//------------------------------------------------------------------------------		

		std::map<double, std::vector<IndexPair>> ordered_neighs;
		if (export_per_baseline)
		{
			ordered_neighs = neighbors_by_rings(mfpc.mia(), ck, cl, maxv, 2., 12.);		
		}
		else
		{
			ordered_neighs[0.] = neighbors(mfpc.mia(), ck, cl, maxv, 2., 12.); //FIXED NEIGBORHOOD?
		}
		
		for(const auto& [baseline, neighs]: ordered_neighs)
		{
	//------------------------------------------------------------------------------
	// Get Observations 				
	//------------------------------------------------------------------------------
			PRINT_INFO("=== Pairing observations from ("<<ck<<", "<<cl<< ") with mode ("<< mode <<")...");
			make_functors(functors, neighs, ck, cl, mfpc, scene, mode);
			
			GUI(
				for (auto [nk, nl] : neighs)
				{
					const P2D c = mfpc.mia().nodeInWorld(ck,cl);
					const double r = (c - mfpc.mia().nodeInWorld(nk,nl)).norm();
		  			Viewer::context().layer(Viewer::layer())
		  				.pen_color(v::green).pen_width(2)
		  				.add_circle(c[0], c[1], r)
		  				.update();
		  		}
			);	
	//------------------------------------------------------------------------------
	// Compute costs				
	//------------------------------------------------------------------------------
			//evaluate observations, find min cost
			minv = 2.01;
			const double stepv = (maxv - minv) / nbsample;
			maxv = maxv + 10.; //goes beyond to eliminate wrong hypotheses
		
			functors.shrink_to_fit();
			std::vector<PnD<16>> data;
			
			PRINT_INFO("=== Computing cost function at ("<<ck<<", "<<cl<< ") with baseline (B = "<< baseline <<")...");
			compute_costs(data, functors, mfpc, minv, maxv, stepv);

	//------------------------------------------------------------------------------
	// Save costs				
	//------------------------------------------------------------------------------		
			PRINT_INFO("=== Saving cost function...");
			std::ostringstream oss;
			
			for (auto& d: data)
			{ 
				int i = 0;
				if (d[4] == 0.) continue; //if no observations: continue
				oss << d[i++] 	<< "," << d[i++] 	<< "," << d[i++] 	<< "," << baseline << "," //v, z, depth, baseline
					<< d[i++] 	<< "," << d[i++] 	<< "," << d[i++] 	<< "," //rmse, cost, nobs
					<< d[i++] 	<< "," << d[i++] 	<< "," << d[i++] 	<< "," //median, mean, sigma
					<< d[i++] 	<< "," << d[i++] 	<< "," << d[i++]	<< "," << d[i++] << "," //normalized weighed cost, mean weighed cost, wrmse, weight
					<< d[i++] 	<< "," << d[i++] 	<< "," << d[i] 		<< std::endl; //mu_err, sigma_err, scaled sigma
			}
			
			ofscf << oss.str();
	
			if (export_per_baseline)
			{
				std::ofstream ofs(
					"costfunction-" + std::to_string(getpid())
					+ "-at-" + std::to_string(ck) + "-" + std::to_string(cl)
					+ "-baseline-" + std::to_string(baseline) + ".csv"
				); 
				if (not ofs.good()) throw std::runtime_error(std::string("Cannot open file costfunction.csv"));
				
				ofs << headercsv.str();				
				ofs << oss.str();
				
				ofs.close();
			}
//------------------------------------------------------------------------------
		}
		ofscf.close();
//------------------------------------------------------------------------------
	}, vfunctor);
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
void export_cost_function_from_obs(
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations,
	double minv, double maxv, double nbsample	
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
		
//------------------------------------------------------------------------------
		std::ofstream ofscf(
			"costfunction-frame-" + std::to_string(observations[0].frame) + "-" + std::to_string(getpid())+ ".csv"
		); 
		if (not ofscf.good()) throw std::runtime_error(std::string("Cannot open file costfunction.csv"));
		
		std::ostringstream headercsv;
		headercsv << "v,z,depth,baseline,"
				<< "rmse,cost,nbobs,"
				<< "median,mean,sigma,"
				<< "nwcost,mwcost,wrmse,weight,"
				<< "muerr,sigmaerr,ssigmaerr\n";
		
		ofscf << headercsv.str();
		
	//------------------------------------------------------------------------------
	// Get functors 				
	//------------------------------------------------------------------------------
		PRINT_INFO("=== Pairing observations from BAP observations...");
		make_functors_from_obs(functors, mfpc, scene, observations);
		DEBUG_VAR(functors.size());
		
	//------------------------------------------------------------------------------
	// Compute costs				
	//------------------------------------------------------------------------------
		//evaluate observations, find min cost
		minv = 2.01;
		const double stepv = (maxv - minv) / nbsample;
		maxv = maxv + 5.; //goes beyond to eliminate wrong hypotheses
	
		functors.shrink_to_fit();
		std::vector<PnD<16>> data;
		
		PRINT_INFO("=== Computing cost function from observations...");
		compute_costs(data, functors, mfpc, minv, maxv, stepv);
	
	//------------------------------------------------------------------------------
	// Save costs				
	//------------------------------------------------------------------------------		
		PRINT_INFO("=== Saving cost function...");
		std::ostringstream oss;
		
		for (auto& d: data)
		{ 
			int i = 0;
			if (d[4] == 0.) continue; //if no observations: continue
			oss << d[i++] 	<< "," << d[i++] 	<< "," << d[i++] 	<< "," << 0. << "," //v, z, depth, baseline
				<< d[i++] 	<< "," << d[i++] 	<< "," << d[i++] 	<< "," //rmse, cost, nobs
				<< d[i++] 	<< "," << d[i++] 	<< "," << d[i++] 	<< "," //median, mean, sigma
				<< d[i++] 	<< "," << d[i++] 	<< "," << d[i++]	<< "," << d[i++] << "," //normalized weighed cost, mean weighed cost, wrmse, weight
				<< d[i++] 	<< "," << d[i++] 	<< "," << d[i] 		<< std::endl; //mu_err, sigma_err, scaled sigma
		}
		
		ofscf << oss.str();
		ofscf.close();	
			
	}, vfunctor);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void export_depth_histogram(
	const RawCoarseDepthMap& dm
) 
{
	std::ofstream ofs(
		"depthisto-" + std::to_string(getpid())+".csv"
	); 
	if (not ofs.good()) throw std::runtime_error(std::string("Cannot open file costfunction.csv"));
	
	std::ostringstream headercsv;
	headercsv << "k,l,depth\n";
	
	ofs << headercsv.str();
	
	std::ostringstream oss;
	
	for (std::size_t k = 0; k < dm.mia().width(); ++k)
	{
		for (std::size_t l = 0; l < dm.mia().height(); ++l)
		{
			oss << k << "," << l << "," << dm.depth(k,l) << "\n";			
		}
	}
	
	ofs << oss.str();
	ofs.close();		
}

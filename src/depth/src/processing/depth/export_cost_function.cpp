#include "export_cost_function.h"

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
  				.add_circle(c[0], c[1], 16.5)
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


void export_cost_function(
	const PlenopticCamera& mfpc, const Image& scene, 
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
	
	
	const auto [ck, cl] = extract_micro_image_indexes(scene, mfpc.mia());
	std::vector<IndexPair> neighs = neighbors(mfpc.mia(), ck, cl, maxv); 
		
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
						refview, targetview,
						ref, target,
						mfpc
				);			 	
		 	}
		}
		else if (mode == ObservationsParingStrategy::ALL_PAIRS)
		{
			const std::size_t n = neighs.size() * (neighs.size() - 1) / 2;
			std::vector<MicroImage> vmi; vmi.reserve(n);
			std::vector<Image> vview; vview.reserve(n);
			
			vmi.emplace_back(ref); vview.emplace_back(refview);
			
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
				
				vmi.emplace_back(target); vview.emplace_back(targetview);
			}

			//for each observation
			for(std::size_t i = 0; i < vmi.size(); ++i)
			{		
				for(std::size_t j = i+1; j < vmi.size(); ++j)
				{
					//add in solver
					functors.emplace_back(
							vview[i], vview[j],
							vmi[i], vmi[j],
							mfpc
					);
				}
			}	
		}
		else
		{
			DEBUG_ASSERT(false, "Can't initialize depth, no other strategy implemented yet");
		}
		
	 	//evaluate observations, find min cost
		const double stepv = (maxv - minv) / nbsample;
		maxv = maxv + 5.*stepv; //goes beyond to eliminate wrong hypotheses
	
		functors.shrink_to_fit();
		std::vector<P3D> xys; xys.reserve(functors.size());
		
		PRINT_INFO("=== Computing cost function at ("<<ck<<", "<<cl<< ") ...");
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
			xys.emplace_back(v, mfpc.v2obj(v), cost.get());
		}
		
		PRINT_INFO("=== Saving cost function...");
		std::ofstream ofs("costfunction-"+std::to_string(getpid())+"-at-"+std::to_string(ck)+"-"+std::to_string(cl)+".csv"); 
		if (!ofs.good())
			throw std::runtime_error(std::string("Cannot open file costfunction.csv"));
		
		ofs << "v,depth,cost\n";
		for(auto& xy: xys) ofs << xy[0] << "," << xy[1]  << "," << xy[2]<< std::endl;
		
		ofs.close();	

	}, vfunctor);
}


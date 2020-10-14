#include "pairing.h"

template <typename Functors>
void make_functors(
	Functors& functors, const std::vector<IndexPair>& neighs,
	std::size_t ck, std::size_t cl,
	const PlenopticCamera& mfpc, const Image& scene, 
	ObservationsPairingStrategy mode
)
{
	functors.clear();
	
	const std::size_t I = mfpc.I();
	const int W = std::ceil(mfpc.mia().diameter());
	
	//compute ref observation	 	
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
 
 	if (mode == ObservationsPairingStrategy::CENTRALIZED)
 	{	
 		functors.reserve(neighs.size());
 		
	 	//for each neighbor, create observation
	 	for (auto [nk, nl] : neighs)
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
	else if (mode == ObservationsPairingStrategy::ALL_PAIRS)
	{
		const std::size_t n = neighs.size() + 1;
		std::vector<MicroImage> vmi; vmi.reserve(n);
		std::vector<Image> vview; vview.reserve(n);
		
		vmi.emplace_back(ref); vview.emplace_back(refview);
		
		for (auto [nk, nl] : neighs)
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
		
		functors.reserve(neighs.size() * (neighs.size() - 1) / 2);
		
		//for each observation
		for (std::size_t i = 0; i < vmi.size(); ++i)
		{		
			for (std::size_t j = i+1; j < vmi.size(); ++j)
			{
				//add in solver
				functors.emplace_back(
					vview[i], vview[j],
					vmi[i], vmi[j],
					mfpc
				);
			}
		}
		functors.shrink_to_fit();	
	}
	else
	{
		DEBUG_ASSERT(false, "Can't pair observations, no other strategy implemented yet");
	}
}


template <typename Functors>
void make_functors_from_obs(
	Functors& functors,
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
)
{
	functors.clear();

	///split observations according to cluster index
	std::unordered_map<Index /* cluster index */, BAPObservations> clusters;
	for (const auto& ob : observations)
		clusters[ob.cluster].push_back(ob);	
	
	///for each cluster
	for (auto & [cluster, obs_] : clusters)
	{
		std::vector<IndexPair> neighs; neighs.reserve(obs_.size());
		
		for (const auto& ob : obs_) neighs.emplace_back(ob.k, ob.l);
		
		const auto [ck, cl] = neighs[0];
		
		Functors fnctrs;
		make_functors(fnctrs, neighs, ck, cl, mfpc, scene, ObservationsPairingStrategy::ALL_PAIRS);	
		
		for (auto &&f: fnctrs) functors.emplace_back(f);
	}
	
	functors.shrink_to_fit();
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
#include "optimization/errors/blurawaredisp.h" //BlurAwareDisparityCostError
#include "optimization/errors/disparity.h" //DisparityCostError

template void make_functors(
	std::vector<BlurAwareDisparityCostError>& functors, const std::vector<IndexPair>& neighs,
	std::size_t ck, std::size_t cl,
	const PlenopticCamera& mfpc, const Image& scene, 
	ObservationsPairingStrategy mode
);

template void make_functors(
	std::vector<DisparityCostError>& functors, const std::vector<IndexPair>& neighs,
	std::size_t ck, std::size_t cl,
	const PlenopticCamera& mfpc, const Image& scene, 
	ObservationsPairingStrategy mode
);

template void make_functors_from_obs(
	std::vector<BlurAwareDisparityCostError>& functors,
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
);

template void make_functors_from_obs(
	std::vector<DisparityCostError>& functors,
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
);

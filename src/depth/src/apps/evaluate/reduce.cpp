#include "reduce.h"

#include <algorithm>

#include <pleno/processing/tools/stats.h>


std::map<Index, double> reduce(const std::map<Index, XYZs>& maps)
{
	std::map<Index, double> dists;
	
	for (const auto& [frame, xyzs] : maps)
	{
		std::vector<double> zs; zs.reserve(xyzs.size());
		std::transform(
			xyzs.begin(), xyzs.end(),
			std::back_inserter(zs),
			[](const auto&p) -> double { return p.z; }
		);
		
		dists[frame] = median(zs);
	}
	
	return dists;
}

std::map<Index, double> reduce(const std::map<Index, PointCloud>& maps)
{
	std::map<Index, double> dists;
	
	for (const auto& [frame, pc] : maps)
	{
		std::vector<double> zs; zs.reserve(pc.size());
		std::transform(
			pc.features().begin(), pc.features().end(),
			std::back_inserter(zs),
			[](const auto&p) -> double { return p.z(); }
		);
		
		dists[frame] = median(zs);
	}
	
	return dists;
}


std::map<Index, double> reduce(const std::map<Index, Pose>& maps)
{
	std::map<Index, double> dists;
	
	for (const auto& [frame, pose] : maps)
	{
		dists[frame] = std::fabs(pose.translation()[2]);
	}
	
	return dists;
}

std::map<Index, double> reduce(const std::map<Index, RawDepthMap>& maps, const PlenopticCamera& pcm)
{
	std::map<Index, double> dists;
		
	for (const auto& [frame, dm] : maps)
	{
		std::vector<double> vs; vs.reserve(dm.width() * dm.height());
		
		for (std::size_t k = 0; k < dm.width(); ++k)
		{
			for (std::size_t l = 0; l < dm.height(); ++l)
			{
				if (dm.depth(k,l) != DepthInfo::NO_DEPTH)
				{
					vs.emplace_back(dm.depth(k,l));
				}
			}
		}
		
		vs.shrink_to_fit();
		const double vmed =  median(vs);
		
		if (dm.is_virtual_depth()) dists[frame] = pcm.v2obj(vmed);
		else  dists[frame] = vmed;
	}
	
	return dists;
}

std::map<Index, double> reduce(const std::map<Index, RawDepthMap>& maps, const std::unordered_map<Index, BAPObservations> &obs, const PlenopticCamera& pcm)
{
	std::map<Index, double> dists;
	
	for (const auto& [frame, dm] : maps)
	{		
		DEBUG_ASSERT((dm.is_coarse_map()), "Can't reduce refined map with observations.");
		
		const BAPObservations& baps = obs.at(frame);
		
		std::vector<double> vs; vs.reserve(baps.size());
		
		for (const auto& bap : baps)
		{
			vs.emplace_back(dm.depth(bap.k,bap.l));		
		}

		const double vmed =  median(vs);
			
		if (dm.is_virtual_depth()) dists[frame] = pcm.v2obj(vmed);
		else  dists[frame] = vmed;
	}
	
	return dists;
}

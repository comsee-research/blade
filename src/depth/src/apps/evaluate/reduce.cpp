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

std::map<Index, double> reduce(const std::map<Index, Pose>& maps)
{
	std::map<Index, double> dists;
	
	for (const auto& [frame, pose] : maps)
	{
		dists[frame] = std::fabs(pose.translation()[2]);
	}
	
	return dists;
}

std::map<Index, double> reduce(const std::map<Index, RawCoarseDepthMap>& maps)
{
	std::map<Index, double> dists;
	
	bool is_virtual_depth = true;
	
	for (const auto& [frame, dm] : maps)
	{
		is_virtual_depth = is_virtual_depth and dm.is_virtual_depth();
		
		std::vector<double> vs; vs.reserve(dm.mia().width() * dm.mia().height());
		
		for (std::size_t k = 0; k < dm.mia().width(); ++k)
		{
			for (std::size_t l = 0; l < dm.mia().height(); ++l)
			{
				if (dm.depth(k,l) != DepthInfo::NO_DEPTH)
				{
					vs.emplace_back(dm.depth(k,l));
				}
			}
		}
		
		vs.shrink_to_fit();
		const double vmed =  median(vs);
		
		if (is_virtual_depth) dists[frame] = dm.pcm().v2obj(vmed);
		else  dists[frame] = vmed;
	}
	
	return dists;
}

std::map<Index, double> reduce(const std::map<Index, RawCoarseDepthMap>& maps, const std::unordered_map<Index, BAPObservations> &obs)
{
	std::map<Index, double> dists;
	bool is_virtual_depth = true;
	
	for (const auto& [frame, dm] : maps)
	{
		is_virtual_depth = is_virtual_depth and dm.is_virtual_depth();
		
		const BAPObservations& baps = obs.at(frame);
		
		std::vector<double> vs; vs.reserve(baps.size());
		
		for (const auto& bap : baps)
		{
			vs.emplace_back(dm.depth(bap.k,bap.l));		
		}

		const double vmed =  median(vs);
			
		if (is_virtual_depth) dists[frame] = dm.pcm().v2obj(vmed);
		else  dists[frame] = vmed;
	}
	
	return dists;
}
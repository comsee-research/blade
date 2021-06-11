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
		PRINT_DEBUG("frame ("<<frame<<"): med = " << median(zs) << ", mean = " << mean(zs) << ", std = " << stddev(zs));
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
		PRINT_DEBUG("frame ("<<frame<<"): med = " << median(zs) << ", mean = " << mean(zs) << ", std = " << stddev(zs));
	}
	
	return dists;
}

std::map<Index, double> reduce(const std::map<Index, Plane>& maps)
{
	std::map<Index, double> dists;
	
	auto it = maps.begin();
	const Plane& ref = it->second;
	
	const P3D& p0 = ref.origin();
	const double z0 = p0.z();
	
	dists[it->first] = z0;
	++it;
	
	for (; it != maps.end(); ++it)
	{
		const Index frame = it->first;
		const Plane& o = it->second;
						
		dists[frame] = (z0 + o.dist(p0));
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
		std::vector<double> zs; zs.reserve(dm.width() * dm.height());
		
		const RawDepthMap mdm = dm.to_metric(pcm);
		
		for (std::size_t k = 0; k < dm.width(); ++k)
		{
			for (std::size_t l = 0; l < dm.height(); ++l)
			{
				if (mdm.depth(k,l) != DepthInfo::NO_DEPTH)
				{
					zs.emplace_back(mdm.depth(k,l));
				}
			}
		}
		
		zs.shrink_to_fit();
		
		dists[frame] = median(zs);
		PRINT_DEBUG("frame ("<<frame<<"): med = " << median(zs) << ", mean = " << mean(zs) << ", std = " << stddev(zs));
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
		
		std::vector<double> zs; zs.reserve(baps.size());
		
		const RawDepthMap mdm = dm.to_metric(pcm);
		
		for (const auto& bap : baps)
		{
			zs.emplace_back(mdm.depth(bap.k,bap.l));		
		}
		
		dists[frame] = median(zs);
		PRINT_DEBUG("frame ("<<frame<<"): med = " << median(zs) << ", mean = " << mean(zs) << ", std = " << stddev(zs));
	}
	
	return dists;
}

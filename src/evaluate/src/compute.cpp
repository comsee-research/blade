#include "compute.h"

#include <vector>

#include <pleno/io/printer.h>
#include <pleno/processing/tools/stats.h>

DepthError compute(const std::map<Index, double>& depths, const std::map<Index, double>& gtdepth)
{
	std::map<double, std::vector<double>> absdepth;
	std::map<double, double>			  dirdepth;
	std::map<double, std::vector<double>> reldepth;
	
	for (const auto& [f, d] : depths)
	{
		PRINT_DEBUG("f ("<<f<<") = " << d);
	}
	
	for (auto it = gtdepth.cbegin(); it != gtdepth.cend(); ++it)
	{
		for (auto nit = std::next(it, 1); nit != gtdepth.cend(); ++nit)	
		{
			if (depths.count(it->first) == 0 or depths.count(nit->first) == 0) continue;
			
			const double delta = std::fabs(it->second - nit->second);
			const double dz = std::fabs(depths.at(it->first) - depths.at(nit->first));
			
			absdepth[delta].push_back(std::fabs(delta - dz));
			reldepth[delta].push_back(100. * std::fabs(delta-dz) / delta);				
		}	
	}
		
	auto itref = gtdepth.cbegin(); 
	for (; itref != gtdepth.cend(); ++itref)
	{
		if (itref->second == 0.) break;	
	}	
	
	for (auto nit = gtdepth.cbegin(); nit != gtdepth.cend(); ++nit)	
	{
		if (depths.count(nit->first) == 0) continue;
		if (nit == itref) continue;
		
		const double delta = std::fabs(itref->second - nit->second);
		const double dz = std::fabs(depths.at(itref->first) - depths.at(nit->first));
		
		dirdepth[delta] = dz;
	}
	
	DepthError derr;

	PRINT_INFO("=== Compute absolute errors");
	for (auto & [d, dzs] : absdepth)
	{
		double emean = mean(dzs);
		double estd = dzs.size() > 1 ? stddev(dzs) : 0.;
		
		PRINT_DEBUG("d(" << d << "): err = " << emean << ", std = " << estd);
		derr.abserr[d] = {emean, estd};	
	}
	
	PRINT_INFO("=== Compute direct depths");
	for (auto & [d, dz] : dirdepth)
	{		
		PRINT_DEBUG("d_gt(" << d << "): d_z = " << dz);
		derr.dirdepth[d] = dz;	
	}		
	
	PRINT_INFO("=== Compute relative errors");
	for (auto & [d, dzs] : reldepth)
	{
		double emean = mean(dzs);
		double estd = dzs.size() > 1 ? stddev(dzs) : 0.;
		
		PRINT_DEBUG("d(" << d << "): err = " << emean << ", std = " << estd);
		derr.relerr[d] = {emean, estd};	
	}	
	
	return derr;
}

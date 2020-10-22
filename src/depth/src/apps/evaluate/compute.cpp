#include "compute.h"

#include <vector>

#include <pleno/io/printer.h>
#include <pleno/processing/tools/stats.h>

DepthError compute(const std::map<Index, double>& depths, const std::map<Index, double>& gtdepth)
{
	std::map<double, std::vector<double>> absdepth;
	std::map<double, std::vector<double>> reldepth;
	
	for (const auto& [f, d] : depths)
	{
		PRINT_DEBUG("f ("<<f<<") = " << d);
	}
	
	for (auto it = gtdepth.cbegin(); it != gtdepth.cend(); ++it)
	{
		for (auto nit = std::next(it, 1); nit != gtdepth.cend(); ++nit)	
		{
			const double delta = std::fabs(it->second - nit->second);
			const double dz = std::fabs(depths.at(it->first) - depths.at(nit->first));
			
			absdepth[delta].push_back(std::fabs(delta - dz));
			reldepth[delta].push_back(100. * std::fabs(delta-dz) / delta);				
		}	
	}
	
	DepthError derr;

	PRINT_INFO("=== Compute absolute errors");
	for (auto & [d, dzs] : absdepth)
	{
		double emean = mean(dzs);
		double estd = dzs.size() > 1 ? stddev(dzs) : 0.;
		
		PRINT_DEBUG("d("<<d<<"): err = " << emean << ", std = " << estd);
		derr.abserr[d] = {emean, estd};	
	}	
	
	PRINT_INFO("=== Compute relative errors");
	for (auto & [d, dzs] : reldepth)
	{
		double emean = mean(dzs);
		double estd = dzs.size() > 1 ? stddev(dzs) : 0.;
		
		PRINT_DEBUG("d("<<d<<"): err = " << emean << ", std = " << estd);
		derr.relerr[d] = {emean, estd};	
	}	
	
	return derr;
}

#include "export.h"

#include <iostream>

#include <pleno/io/printer.h>

void export_depth_errors(const DepthError& derr)
{
	PRINT_DEBUG("Exporting errors...");
	{
		std::ofstream ofs(
			"depth-errors-" + std::to_string(getpid())+".csv"
		); 
		if (not ofs.good()) throw std::runtime_error(std::string("Cannot open file errors.csv"));
		
		std::ostringstream headercsv;
		headercsv << "dist,abserr,absstd,relerr,relstd\n";
		
		ofs << headercsv.str();
		
		std::ostringstream oss;
		
		for (const auto& [dist, abspairerr] : derr.abserr)
		{
			const auto& [abserr, absstd] = abspairerr;
			const auto& [relerr, relstd] = derr.relerr.at(dist);
			
			oss << dist << "," << abserr << "," << absstd << "," << relerr << "," << relstd << "\n"; 	
		}
		
		ofs << oss.str();
		ofs.close();
	}
	
	PRINT_DEBUG("Exporting depths...");
	{
		std::ofstream ofs(
			"depth-measured-" + std::to_string(getpid())+".csv"
		); 
		if (not ofs.good()) throw std::runtime_error(std::string("Cannot open file errors.csv"));
		
		std::ostringstream headercsv;
		headercsv << "dgt,dz\n";
		
		ofs << headercsv.str();
		
		std::ostringstream oss;
		
		for (const auto& [dgt, dz] : derr.dirdepth)
		{			
			oss << dgt << "," << dz << "\n"; 	
		}
		
		ofs << oss.str();
		ofs.close();
	}
	
}


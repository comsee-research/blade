#include "utils.h"

// Boost
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

//libpleno
#include <pleno/io/printer.h>

//blade
#include "processing/depth/strategy.h"

Config_t parse_args(int argc, char *argv[])
{
	namespace po = boost::program_options;

	po::options_description desc("Options");
	
	desc.add_options()
		("help,h", "Print help messages")
		("gui,g", 
			po::value<bool>()->default_value(true),
			"Enable GUI (image viewers, etc.)"
		)
		("verbose,v", 
			po::value<bool>()->default_value(true),
			"Enable output with extra information"
		)
		("level,l", 
			po::value<std::uint16_t>()->default_value(Printer::Level::ALL),
			"Select level of output to print (can be combined):\n"
			"NONE=0, ERR=1, WARN=2, INFO=4, DEBUG=8, ALL=15"
		)
		("pcamera,c",
			po::value<std::string>()->default_value(""),
			"Path to camera configuration file"
		)
		("pparams,p",
			po::value<std::string>()->default_value(""),
			"Path to camera internal parameters configuration file"
		)
		("features,f",
			po::value<std::string>()->default_value(""),
			"Path to observations file"
		)
		("dm",
			po::value<std::string>()->default_value(""),
			"Path to depthmaps configuration file"
		)
		("pc",
			po::value<std::string>()->default_value(""),
			"Path to pointclouds configuration file"
		)
		("pl",
			po::value<std::string>()->default_value(""),
			"Path to planes configuration file"
		)
		("csv",
			po::value<std::string>()->default_value(""),
			"Path to csv depthmap configuration file"
		)
		("xyz",
			po::value<std::string>()->default_value(""),
			"Path to xyz pointcloud configuration file"
		)
		("mat",
			po::value<std::string>()->default_value(""),
			"Path to mat poses configuration file"
		)
		("poses",
			po::value<std::string>()->default_value(""),
			"Path to poses configuration file"
		)
		("gt",
			po::value<std::string>()->default_value(""),
			"Path to ground truth displacements file (.csv)"
		);

	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm);
	}
	catch (po::error &e) {
		/* Invalid options */
		std::cerr << "Error: " << e.what() << std::endl << std::endl;
		std::cout << "Depth evaluation with a MFPC:" << std::endl
		  << desc << std::endl;
		exit(0);
	}
	po::notify(vm);
	
	//check if hepl or no arguments then display usage
	if (vm.count("help") or argc==1)
	{
		/* print usage */
		std::cout << "Depth evaluation with a MFPC:" << std::endl
				      << desc << std::endl;
		exit(0);
	}
	
	//check mandatory parameters
	if(	vm["pparams"].as<std::string>() == ""
		or vm["pcamera"].as<std::string>() == ""
		or vm["gt"].as<std::string>() == ""
	)
	{
		/* print usage */
		std::cerr << "Please specify at the configuration files. " << std::endl;
		std::cout << "Depth evaluation with a MFPC:" << std::endl
				      << desc << std::endl;
		exit(0);
	}
	
	
	Config_t config;
	
	config.use_gui 	 		= vm["gui"].as<bool>();
	config.verbose			= vm["verbose"].as<bool>();
	config.level			= vm["level"].as<std::uint16_t>();
	
	config.path.camera 		= vm["pcamera"].as<std::string>();
	config.path.params 		= vm["pparams"].as<std::string>();
	config.path.features 	= vm["features"].as<std::string>();
	
	config.path.dm 			= vm["dm"].as<std::string>();
	config.path.pc 			= vm["pc"].as<std::string>();
	config.path.pl 			= vm["pl"].as<std::string>();
	config.path.csv			= vm["csv"].as<std::string>();
	config.path.xyz 		= vm["xyz"].as<std::string>();
	config.path.poses 		= vm["poses"].as<std::string>();
	config.path.mat 		= vm["mat"].as<std::string>();
	
	config.path.gt 			= vm["gt"].as<std::string>();
	
	return config; 
}


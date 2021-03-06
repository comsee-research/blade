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
		("pimages,i",
			po::value<std::string>()->default_value(""),
			"Path to images configuration file"
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
		("strategy,s",
			po::value<std::string>()->default_value(""),
			"Path to depth estimation strategies configuration file"
		)
		("dm",
			po::value<std::string>()->default_value(""),
			"Path to depthmap file"
		)
		("output,o",
			po::value<std::string>()->default_value("depth.png"),
			"Path to save depth map"
		)
		("save_all", 
			po::value<bool>()->default_value(false),
			"Save automaticaly all depthmap and pointcloud"
		)
		("run_all", 
			po::value<bool>()->default_value(false),
			"Run automaticaly all depthmap estimation"
		);

	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm);
	}
	catch (po::error &e) {
		/* Invalid options */
		std::cerr << "Error: " << e.what() << std::endl << std::endl;
		std::cout << "Depth Estimation with a MFPC:" << std::endl
		  << desc << std::endl;
		exit(0);
	}
	po::notify(vm);
	
	//check if hepl or no arguments then display usage
	if (vm.count("help") or argc==1)
	{
		/* print usage */
		std::cout << "Depth Estimation with a MFPC:" << std::endl
				      << desc << std::endl;
		exit(0);
	}
	
	//check mandatory parameters
	if(	(vm["pimages"].as<std::string>() == "" and vm["pparams"].as<std::string>() == "")
		or vm["pcamera"].as<std::string>() == ""
	)
	{
		/* print usage */
		std::cerr << "Please specify at the configuration files. " << std::endl;
		std::cout << "Depth Estimation with a MFPC:" << std::endl
				      << desc << std::endl;
		exit(0);
	}
	
	
	Config_t config;
	
	config.use_gui 	 		= vm["gui"].as<bool>();
	config.verbose			= vm["verbose"].as<bool>();
	config.level			= vm["level"].as<std::uint16_t>();
	config.path.images 		= vm["pimages"].as<std::string>();
	config.path.camera 		= vm["pcamera"].as<std::string>();
	config.path.params 		= vm["pparams"].as<std::string>();
	config.path.features 	= vm["features"].as<std::string>();
	config.path.output 		= vm["output"].as<std::string>();
	config.path.dm 			= vm["dm"].as<std::string>();
	config.path.strategy	= vm["strategy"].as<std::string>();
	
	config.save_all			= vm["save_all"].as<bool>();
	config.run_all			= vm["run_all"].as<bool>();
	
	return config; 
}


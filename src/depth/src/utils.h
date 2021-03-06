#pragma once

#include <iostream>
#include <string>

struct Config_t {
	bool use_gui;
	bool verbose;
	std::uint16_t level;
		
	struct {
		std::string images;
		std::string camera;
		std::string params;
		std::string features;
		std::string output;
		std::string strategy;
		std::string dm;
	} path;
	
	bool save_all;
	bool run_all;
};

Config_t parse_args(int argc, char *argv[]);

#pragma once

#include <iostream>
#include <string>

struct Config_t {
	bool use_gui;
	bool verbose;
	std::uint16_t level;
	
	bool use_probabilistic;
	std::uint16_t method;
		
	struct {
		std::string images;
		std::string camera;
		std::string params;
		std::string features;
		std::string output;
		std::string dm;
		std::string pc;
	} path;
};

Config_t parse_args(int argc, char *argv[]);

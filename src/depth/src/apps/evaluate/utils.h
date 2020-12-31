#pragma once

#include <iostream>
#include <string>

struct Config_t {
	bool use_gui;
	bool verbose;
	std::uint16_t level;
			
	struct {
		std::string camera;
		std::string params;
		std::string features;
		std::string output;
		std::string dm;
		std::string csv;
		std::string xyz;
		std::string poses;
		std::string mat;
		std::string gt;
	} path;
};

Config_t parse_args(int argc, char *argv[]);

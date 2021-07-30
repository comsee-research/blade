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
		std::string extrinsics;
		std::string pts;
		std::string pc;
		std::string dm;
	} path;
};

Config_t parse_args(int argc, char *argv[]);

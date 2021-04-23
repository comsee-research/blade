#pragma once

#include <pleno/types.h>

struct DepthMapImage {
	Image image;
	cv::Mat colormap;
};

struct xyz 
{
	double x = 0., y = 0., z = 0.;
};

using XYZs = std::vector<xyz>;


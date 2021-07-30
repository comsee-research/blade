#pragma once

#include <pleno/types.h>

struct xyz 
{
	double x = 0., y = 0., z = 0.;
};

using XYZs = std::vector<xyz>;

enum Axis { X = 0, Y = 1, Z = 2, XY = 4, XZ = 5, YZ = 6, XYZ = 7 };


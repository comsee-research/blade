#pragma once

#include <map>

#include <pleno/types.h>
#include "types.h"

struct DepthError {
	using PairError = std::pair<double /*error*/, double /*stddev*/>;
	
	std::map<double /*seq dist*/, PairError> abserr;
	std::map<double /*seq dist*/, PairError> relerr;
	std::map<double /* gt depth */, double /* measured depth */> dirdepth;
};

DepthError compute(const std::map<Index, double>& depths, const std::map<Index, double>& gtdepth);

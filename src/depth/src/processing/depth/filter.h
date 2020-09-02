#pragma once

#include "geometry/depth/RawCoarseDepthMap.h"

RawCoarseDepthMap median_filter_depth(const RawCoarseDepthMap& dm, double size = 3.5);

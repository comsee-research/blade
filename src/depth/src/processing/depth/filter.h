#pragma once

#include "geometry/depth/RawCoarseDepthMap.h"

constexpr double AUTOMATIC_FILTER_SIZE = -1.;

RawCoarseDepthMap median_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_median_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);

RawCoarseDepthMap erosion_filter_depth(const RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE);
void inplace_erosion_filter_depth(RawCoarseDepthMap& dm, double size = AUTOMATIC_FILTER_SIZE); 
 
RawCoarseDepthMap minmax_filter_depth(const RawCoarseDepthMap& dm, double min, double max); 
void inplace_minmax_filter_depth(RawCoarseDepthMap& dm, double min, double max); 

#pragma once

#include <map>
#include <vector>

#include <pleno/types.h>
#include "../../types.h"

#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/observation.h>
#include <pleno/geometry/pose.h>

#include "geometry/depth/RawDepthMap.h"

std::map<Index, double> reduce(const std::map<Index, XYZs>& maps);
std::map<Index, double> reduce(const std::map<Index, Pose>& maps);
std::map<Index, double> reduce(const std::map<Index, RawDepthMap>& maps);
std::map<Index, double> reduce(const std::map<Index, RawDepthMap>& maps, const std::unordered_map<Index, BAPObservations> &obs);

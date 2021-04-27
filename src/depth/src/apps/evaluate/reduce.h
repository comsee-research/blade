#pragma once

#include <map>
#include <vector>

#include <pleno/types.h>
#include "../../types.h"

#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/observation.h>
#include <pleno/geometry/pose.h>

#include "geometry/depth/RawDepthMap.h"
#include "geometry/depth/PointCloud.h"

std::map<Index, double> reduce(const std::map<Index, XYZs>& maps);

std::map<Index, double> reduce(const std::map<Index, Pose>& maps);

std::map<Index, double> reduce(const std::map<Index, PointCloud>& maps);

std::map<Index, double> reduce(const std::map<Index, Plane>& maps);

std::map<Index, double> reduce(
	const std::map<Index, RawDepthMap>& maps, const PlenopticCamera& pcm
);

std::map<Index, double> reduce(
	const std::map<Index, RawDepthMap>& maps, 
	const std::unordered_map<Index, BAPObservations> &obs, 
	const PlenopticCamera& pcm
);

#pragma once

#include <map>
#include <vector>

#include <pleno/types.h>
#include "../../types.h"

#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/observation.h>
#include <pleno/geometry/pose.h>

#include "geometry/depth/depthmap.h"
#include "geometry/depth/pointcloud.h"

std::map<Index, double> reduce(const std::map<Index, XYZs>& maps);

std::map<Index, double> reduce(const std::map<Index, Pose>& maps);

std::map<Index, double> reduce(const std::map<Index, PointCloud>& maps);

std::map<Index, double> reduce(const std::map<Index, Plane>& maps);

std::map<Index, double> reduce(
	const std::map<Index, DepthMap>& maps, const PlenopticCamera& pcm
);

std::map<Index, double> reduce(
	const std::map<Index, DepthMap>& maps, 
	const std::unordered_map<Index, BAPObservations> &obs, 
	const PlenopticCamera& pcm
);

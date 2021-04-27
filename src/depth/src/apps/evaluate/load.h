#pragma once

#include <map>
#include <iostream>

#include <pleno/types.h>
#include "../../types.h"

#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/plane.h>

#include "geometry/depth/RawDepthMap.h"
#include "geometry/depth/PointCloud.h"

#include <pleno/io/printer.h>
#include <pleno/io/cfg/poses.h>

#include "io/cfg/xyzs.h"
#include "io/cfg/mats.h"
#include "io/cfg/depthmaps.h"
#include "io/cfg/pointclouds.h"
#include "io/cfg/planes.h"

//******************************************************************************
XYZs read_xyz(std::string path);
Pose read_mat(std::string path);

//******************************************************************************

std::map<Index, XYZs> 			load(const XYZsConfig& config); //load xyz point cloud from raytrix
std::map<Index, RawDepthMap> 	load(const DepthMapsConfig& config, const PlenopticCamera& mfpc); //load virtual depth map from blade
std::map<Index, RawDepthMap> 	load_from_csv(std::string path, const PlenopticCamera& mfpc); //load metric depth map from pledge
std::map<Index, PointCloud> 	load(const PointCloudsConfig& config); //load pointclouds from blade
std::map<Index, Plane> 			load(const PlanesConfig& config); //load planes from blade
std::map<Index, Pose> 			load(const CalibrationPosesConfig& config); //load extrinsics poses from compote
std::map<Index, Pose> 			load(const MatsConfig& config); //load extrinsics poses from mat files

std::map<Index, double> 			load_gt_dist(std::string path);

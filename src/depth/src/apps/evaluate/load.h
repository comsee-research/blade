#pragma once

#include <map>
#include <iostream>

#include <pleno/types.h>
#include "../../types.h"

#include <pleno/geometry/camera/plenoptic.h>
#include "geometry/depth/RawCoarseDepthMap.h"

#include <pleno/io/printer.h>
#include <pleno/io/cfg/poses.h>

#include "io/cfg/xyzs.h"
#include "io/cfg/depthmaps.h"

//******************************************************************************

XYZs read_xyz(std::string path);

//******************************************************************************

std::map<Index, XYZs> 				load(const XYZsConfig& config);
std::map<Index, RawCoarseDepthMap> 	load(const DepthMapsConfig& config, const PlenopticCamera& mfpc);
std::map<Index, Pose> 				load(const CalibrationPosesConfig& config);

std::map<Index, double> 			load_gt_dist(std::string path);

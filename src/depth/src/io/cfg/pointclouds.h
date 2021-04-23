#pragma once

#include <pleno/io/archive.h>

V_DEFINE_PROPERTIES(PointCloudConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the pointcloud to load")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)

V_DEFINE_PROPERTIES(PointCloudsConfig)
(    
    V_DEFINE_PROPERTY(pointclouds, std::vector<PointCloudConfig>(0), "Pointclouds configurations")
)

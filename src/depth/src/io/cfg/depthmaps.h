#pragma once

#include <pleno/io/archive.h>

V_DEFINE_PROPERTIES(DepthMapConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the depth map to load")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)

V_DEFINE_PROPERTIES(DepthMapsConfig)
(    
    V_DEFINE_PROPERTY(metric, bool(false), "Metric (true) or virtual (false) depth map")
    V_DEFINE_PROPERTY(coarse, bool(true), "Coarse (true) or refined (false) depth map")
    V_DEFINE_PROPERTY(maps, std::vector<DepthMapConfig>(0), "Depth Maps configurations")
)

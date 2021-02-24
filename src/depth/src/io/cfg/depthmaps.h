#pragma once

#include <libv/core/serialization/serializable_properties.hpp>
#include <libv/core/serialization/contiguous_containers.hpp> //support for std::vector

V_DEFINE_PROPERTIES(DepthMapConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the depth map to load")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)

V_DEFINE_PROPERTIES(DepthMapsConfig)
(    
    V_DEFINE_PROPERTY(metric, bool(false), "Metric (true) or virtual (false) depth map")
    V_DEFINE_PROPERTY(coarse, bool(true), "Coarse (true) or dense (false) depth map")
    V_DEFINE_PROPERTY(maps, std::vector<DepthMapConfig>(0), "Depth Maps configurations")
)

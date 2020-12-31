#pragma once

#include <libv/core/serialization/serializable_properties.hpp>
#include <libv/core/serialization/contiguous_containers.hpp> //support for std::vector

V_DEFINE_PROPERTIES(MatConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the pose to load (.mat)")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)

V_DEFINE_PROPERTIES(MatsConfig)
(    
    V_DEFINE_PROPERTY(mats, std::vector<XYZConfig>(0), "Poses to load")
)

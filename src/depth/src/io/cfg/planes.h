#pragma once

#include <pleno/io/archive.h>

V_DEFINE_PROPERTIES(PlaneConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the plane to load")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)

V_DEFINE_PROPERTIES(PlanesConfig)
(    
    V_DEFINE_PROPERTY(planes, std::vector<PlaneConfig>(0), "Planes configurations")
)

#pragma once

#include <pleno/io/archive.h>

V_DEFINE_PROPERTIES(XYZConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the pointcloud to load (.xyz)")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)

V_DEFINE_PROPERTIES(XYZsConfig)
(    
    V_DEFINE_PROPERTY(xyzs, std::vector<XYZConfig>(0), "Pointcloud configurations")
)

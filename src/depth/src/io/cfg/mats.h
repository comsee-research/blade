#pragma once

#include <pleno/io/archive.h>

V_DEFINE_PROPERTIES(MatConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the pose to load (.mat)")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)

V_DEFINE_PROPERTIES(MatsConfig)
(    
    V_DEFINE_PROPERTY(mats, std::vector<XYZConfig>(0), "Poses to load")
)

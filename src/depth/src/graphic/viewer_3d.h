#pragma once

#include <pleno/graphic/gui.h>
#include "geometry/depth/PointCloud.h"

// Displaying a PointCloud in 3D
void viewer_3d(v::ViewerContext&, const PointCloud& pc);

#pragma once

#include <pleno/graphic/gui.h>

#include <pleno/geometry/plane.h>
#include "geometry/depth/pointcloud.h"

// Displaying a PointCloud in 3D
void viewer_3d(v::ViewerContext&, const PointCloud& pc);

// Displaying a Plane
void viewer_3d(v::ViewerContext&, const Plane& plane);

#include "viewer_3d.h"

#include <GL/gl.h>

// Displaying a PointCloud in 3D
void viewer_3d(v::ViewerContext& v, const PointCloud& pc)
{
    v.add_opengl([&pc](){
        constexpr double transparency_value = 0.6;
        glPointSize(1.0);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_POINTS);
            for (std::size_t i = 0; i < pc.nbPoints(); ++i)
            {
            	const P3D& point = pc.feature(i);
            	const RGBA& color = pc.color(i);
            	
            	glColor4f(color.r / 255., color.g / 255., color.b / 255., color.a / 255.);
            	glAddPoint(point);	
            }
        glEnd();
    });
}


void viewer_3d(v::ViewerContext& v, const Plane& plane)
{
    v.add_opengl([&plane](){
        constexpr double transparency_value = 0.6;
        glPointSize(1.0);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);

		Rays3D rays(3); 
		rays.emplace_back(plane.origin(), plane.e1(), RGBA{0., 0., 255., 127.});
		rays.emplace_back(plane.origin(), plane.e2(), RGBA{0., 255., 0., 127.});
		rays.emplace_back(plane.origin(), plane.e3(), RGBA{255., 0., 0., 127.});

        glBegin(GL_LINES);
        	for (const auto& r : rays)
        	{
		    	glColor4f(r.color().r / 255.0,
		                  r.color().g / 255.0,
		                  r.color().b / 255.0,
		                  r.color().a / 255.0
		                 );
		        glAddLine(r);
            }
        glEnd();
    });
}

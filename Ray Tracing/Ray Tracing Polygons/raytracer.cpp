
#define _USE_MATH_DEFINES

#include <iostream>
#include <fstream>
#include <cstddef>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <limits>

#include "geometry.h"


static const float kInfinity = std::numeric_limits<float>::max();
static const float kEpsilon = 1e-8;
static const Vec3f kDefaultBackgroundColor = Vec3f(0.235294, 0.67451, 0.843137);

inline
float clamp(const float &lo, const float &hi, const float &v)
{
    return std::max(lo,std::min(hi,v));
}

struct Options
{
    uint32_t width = 640;
    uint32_t height = 400;
    Matrix44f cameraToWorld;
    float fov;
    Vec3f backgroundColor = kDefaultBackgroundColor;
};

float toRadian(const float angle)
{
    return angle * M_PI / 180;
}

Vec3f castRay(const Vec3f& orig, const Vec3f& dir)
{
    return (dir + Vec3f(1)) * 0.5;
}

void render(const Options& op)
{
 Vec3f* framebuffer = new Vec3f[op.width * op.height];
 Vec3f* pix = framebuffer;

const float AspectRatio = op.width / (float)op.height;
const float scale = tan(toRadian(op.fov * 0.5));

Vec3f orig;
op.cameraToWorld.multVecMatrix(Vec3f(0),orig);
          
    for(uint32_t j = 0 ; j < op.height; ++j)
    {
        for(uint32_t i = 0; i < op.width; ++i)
        {
            float x = (2.0 * (i + 0.5) / (float)op.width - 1.0) * scale * AspectRatio;
            float y = (1.0 - 2.0 * (j + 0.5) / (float)op.height ) * scale;
            Vec3f dir;
            op.cameraToWorld.multDirMatrix(Vec3f(x,y,-1.0),dir);
            dir.normalize();
            //*(pix++) = castRay(orig,dir);
            *(pix++) = Vec3f(i / (float)op.width);
        }
    }

    std::ofstream ofs("raytracer_output.ppm",std::ios::out|std::ios::binary);
    assert(ofs);
    ofs << "P6\n";
    ofs << op.width << " " << op.height << "\n255\n";

    for(uint32_t k = 0; k < op.width * op.height; k++)
    {
        char r = (char)(255 * clamp(0,1,framebuffer[k].x));
        char g = (char)(255 * clamp(0,1,framebuffer[k].y));
        char b = (char)(255 * clamp(0,1,framebuffer[k].z));

        ofs << r << g << b;
    }
    
    ofs.close();


    delete [] framebuffer;
}

int main(int argc , char** argv)
{
    Options ops;
    ops.fov = 90;
    
    render(ops);

    return 0;
}
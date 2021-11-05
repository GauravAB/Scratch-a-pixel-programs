#include <cstdio>
#include <cstdlib>
#include <vector>
#include <utility>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <algorithm>

#include "geometry.h"

#ifndef M_PI 
#define M_PI 3.14159265
#endif

const float kInfinity = std::numeric_limits<float>::max();

inline
float clamp(const float& lo, const float& hi, const float& v)
{
	return std::max(lo, std::min(hi, v));
}

inline
float deg2rad(const float &d)
{
	return d * M_PI / 180;
}

struct Options
{
	uint32_t width;
	uint32_t height;
	float fov;
};

class Light
{
public:
	Light() {}
};

class Object
{
public:
	Object() {}
	virtual ~Object() {}
};

Vec3f castRay(
	const Vec3f& orig, const Vec3f& dir,
	const std::vector<std::unique_ptr<Object>>& objects,
	const std::vector<std::unique_ptr<Light>>& lights,
	const Options& options,
	uint32_t depth)
{
	Vec3f hitColor = (dir + Vec3f(1)) * 0.5;
	return hitColor;
}

void render(
	const Options& options,
	const std::vector<std::unique_ptr<Object>>& objects,
	const std::vector<std::unique_ptr<Light>>& lights)
{
	Matrix44f cameraToWorld;
	Vec3f* framebuffer = new Vec3f[options.width * options.height];
	Vec3f* pix = framebuffer;
	float scale = tan(deg2rad(options.fov * 0.5));
	float imageAspectRatio = options.width / (float)options.height;
	Vec3f orig;

	cameraToWorld.multVecMatrix(Vec3f(0), orig);
	
	for (uint32_t j = 0; j < options.height; ++j)
	{
		for (uint32_t i = 0; i < options.width; ++i)
		{
			//to camera space 
			float cameraX = ((2.0 * (i + 0.5) / (float)options.width) - 1) * imageAspectRatio * scale;
			float cameraY = (1.0 - 2.0 * (j + 0.5) / (float)options.height) * scale;
			float cameraZ = -1.0;
			Vec3f dir;
			cameraToWorld.multVecMatrix(Vec3f(cameraX, cameraY, cameraZ), dir);
			dir.normalize();

			//*(pix++) = castRay(orig, dir, objects, lights, options, 1);
			*(pix++) = Vec3f(i / (float)options.width);
		}
	}

	//save back to PPM
	std::ofstream ofs;
	ofs.open("output1.ppm", std::ios::out | std::ios::binary);
	ofs << "P6\n" << options.width << " " << options.height << "\n255\n";

	for (uint32_t i = 0; i < options.width * options.height; ++i)
	{
		char r = (char)(255 * clamp(0, 1, framebuffer[i].x));
		char g = (char)(255 * clamp(0, 1, framebuffer[i].y));
		char b = (char)(255 * clamp(0, 1, framebuffer[i].z));
		ofs << r << g << b;
	}
	ofs.close();

	delete[] framebuffer;
}




int main()
{
	//create a scene
	std::vector<std::unique_ptr<Object>> objects;
	std::vector<std::unique_ptr<Light>> lights;

	//setting up options
	Options options;
	options.width = 640;
	options.height = 480;
	options.fov = 90;

	//finally
	render(options, objects, lights);
	return 0;
}
















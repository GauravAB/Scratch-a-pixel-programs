
//USE -DMOLLER_TRUMBORE -DCULLING to optimize the runtime

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <utility>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <random>
#include <chrono>
#include <cassert>

#include <algorithm>

#include "geometry.h"

#define M_PI 3.14159265

constexpr float kEpsilon = 1e-8;

inline
float deg2rad(const float d)
{
	return (d * M_PI / 180);
}

inline
float clamp(const float& lo, const float& hi, const float& v)
{
	return std::max(lo, std::min(hi, v));
}

bool rayTriangleIntersect(
	const Vec3f& orig, const Vec3f& dir,
	const Vec3f& v0, const Vec3f& v1, const Vec3f& v2,
	float &t, float &u, float& v)
{
#ifdef MOLLER_TRUMBORE
	Vec3f v0v1 = v1 - v0;
	Vec3f v0v2 = v2 - v0;
	Vec3f pvec = dir.crossProduct(v0v2);
	float det = v0v1.dotProduct(pvec);
#ifdef CULLING
	//if det is negetive than triangle is backfacing
	//if the det is close to 0, the ray misses the triangle
	if (det < kEpsilon) return false;
#else
	if (fabs(det) < kEpsilon) return false;
#endif
	float invDet = 1 / det;
	Vec3f tvec = orig - v0;
	u = tvec.dotProduct(pvec) * invDet;
	if (u < 0 || u > 1) return false;
	Vec3f qvec = tvec.crossProduct(v0v1);
	v = dir.dotProduct(qvec) * invDet;
	//barycentric coordinates invarient
	if (v < 0 || u + v > 1) return false;
	t = v0v2.dotProduct(qvec) * invDet;

	if( (u + v + (1.0 - (u + v))) != 1.0 )
	{
		std::cout << "u v w " << u << " " << v << " " << (1.0 - u - v) <<   " = " << (float)(u + v + (1.0 - (u + v))) << std::endl;
	}
	

	return true;
#else
	//compute plane's normal
	Vec3f v0v1 = v1 - v0;
	Vec3f v0v2 = v2 - v0;
	// no need to normalize
	Vec3f N = v0v1.crossProduct(v0v2); //N
	float denom = N.dotProduct(N);

	//Step 1: finding P
	//check if ray and plane are parallel
	float NdotRayDirection = N.dotProduct(dir);
	if (fabs(NdotRayDirection) < kEpsilon) // almost 0
		return false;

	//compute d parameter using equation 2
	float d = N.dotProduct(v0);

	//compute t
	t = (N.dotProduct(orig) + d) / NdotRayDirection;

	//check if the triangle is behind the ray
	if (t < 0) return false; //The triangle lies behind the ray

	//compute the intersection point using equation 1
	Vec3f P = orig + t * dir;

	//Step 2: inside-outside test
	Vec3f C;
	//edge0
	Vec3f edge0 = v1 - v0;
	Vec3f vp0 = P - v0;
	C = edge0.crossProduct(vp0);
	if (N.dotProduct(C) < 0) return false; //P is on the right side

	//edge 1
	Vec3f edge1 = v2 - v1;
	Vec3f vp1 = P - v1;
	C = edge1.crossProduct(vp1);
	if ( (u = N.dotProduct(C)) < 0) return false;

	//edge 2
	Vec3f edge2 = v0 - v2;
	Vec3f vp2 = P - v2;
	C = edge2.crossProduct(vp2);
	if ((v = N.dotProduct(C)) < 0) return false;

	u /= denom;
	v /= denom;

	return true;
#endif
}



int main()
{
	Vec3f v0(-1, -1, -5);
	Vec3f v1(1, -1, -5);
	Vec3f v2(0, 1, -5);
	
	const uint32_t width = 640;
	const uint32_t height = 480;
	Vec3f cols[3] = { {0.6,0.4,0.1}, {0.1,0.5,0.3},{0.1,0.3,0.7} };
	Vec3f* framebuffer = new Vec3f[width * height];
	Vec3f* pix = framebuffer;
	float fov = 51.52;
	float scale = tan(deg2rad(fov * 0.5));
	float imageAspectRatio = width / (float)height;
	Vec3f orig(0);

	auto t_start = std::chrono::high_resolution_clock::now();

	for (uint32_t j = 0; j < height; ++j)
	{
		for (uint32_t i = 0; i < width; ++i)
		{
			float x = (2 * (i + 0.5) / (float)width - 1) * imageAspectRatio * scale;
			float y = (1 - 2 * (j + 0.5) / (float)height) * scale;
			Vec3f dir(x, y, -1);
			dir.normalize();
			float t, u, v; 
			if (rayTriangleIntersect(orig, dir, v0, v1, v2, t, u, v))
			{
				*pix = u * cols[0] + v * cols[1] + (1 - u - v) * cols[2];
			}
			pix++;
		}
	}
	auto t_end = std::chrono::high_resolution_clock::now();

	std::ofstream ofs("out.ppm", std::ios::out, std::ios::binary);
	ofs << "P6\n" << width << " " << height << "\n255\n";
	for (uint32_t i = 0; i < height * width; ++i)
	{
		char r = (char)(255 * clamp(0,1,framebuffer[i].x));
		char g = (char)(255 * clamp(0,1,framebuffer[i].y));
		char b = (char)(255 * clamp(0,1,framebuffer[i].z));
		
		ofs << r << g << b;
	}
	ofs.close();
	delete[] framebuffer;

	auto ptime = std::chrono::duration<double, std::milli>(t_end - t_start).count();
	std::cerr << "time required for one triangle: " << ptime << std::endl;

	return 0;
}


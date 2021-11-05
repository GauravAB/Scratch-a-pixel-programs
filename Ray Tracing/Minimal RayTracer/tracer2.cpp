#include <iostream>
#include <fstream>
#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <algorithm>
#include <cassert>
#include <random>
#include <vector>
#include <memory>
#include <utility>
#include <cmath>
#include <limits>
#include <chrono>


#include "geometry.h"



#ifndef M_PI
#define M_PI 3.14159265
#endif

const float kInfinity = std::numeric_limits<float>::max();
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(0, 1);


inline
float deg2rad(const float& d)
{
	return d * M_PI / 180;
}

inline
Vec3f mix(const Vec3f& a, const Vec3f& b, const float& mixValue)
{
	return a * (1 - mixValue) + b * mixValue;
}

inline
float clamp(const float& lo, const float& hi, const float& v)
{
	return std::max(lo, std::min(hi, v));
}

struct Options
{
	uint32_t width;
	uint32_t height;
	float fov;
	Matrix44f cameraToWorld;
};


class Object
{
public:
	Object() : color(dis(gen), dis(gen), dis(gen)) {}
	virtual ~Object() {}
	//The method to compute the intersection of the object with a ray 
	//Returns true if an intersection was found, false otherwise
	//Method could be implemented by any Geometric structure which may be polygonal or mesh structure;
	virtual bool intersect(const Vec3f&, const Vec3f&, float&) const = 0;

	//Method to compute normal and texture coordinates at the surface of the object
	virtual void getSurfaceData(const Vec3f&, Vec3f&, Vec2f&) const = 0;

	Vec3f color;
};

bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1)
{
	float discr = b * b - 4 * a * c;
	if (discr < 0) 
	{
		return false;
	}
	else if (discr == 0)
	{
		x0 = x1 = -0.5 * b / a;
	}
	else
	{
		float q = (b > 0) ?
			-0.5 * (b + sqrt(discr)) :
			-0.5 * (b - sqrt(discr));
		x0 = q / a;
		x1 = c / q;
	}

	return true;
}

//class Sphere is a type of Object which is hittable in my engine
class Sphere : public Object
{
public:
	Sphere(const Vec3f& c, const float& r) : radius(r), radius2(r* r), center(c) {}

	//Ray sphere intersection test
	bool intersect(const Vec3f& orig, const Vec3f& dir, float& t) const
	{
		float t0, t1; //solutions for t if the ray intersects 

		/*
		//geometric solution
		Vec3f L = center - orig;
		float tca = L.dotProduct(dir);
		if (tca < 0)
		{
		//	std::cout << "tca " << tca << std::endl;
			return false;
		}

		float d2 = L.dotProduct(L) - tca * tca;
		if (d2 > radius2) return false;
		float thc = sqrt(radius2 - d2);
		t0 = tca - thc;
		t1 = tca + thc;
		*/

		//analytic solution
		Vec3f L = orig - center;
		float a = dir.dotProduct(dir);
		float b = 2 * dir.dotProduct(L);
		float c = L.dotProduct(L) - radius2;
		if (!solveQuadratic(a, b, c, t0, t1)) return false;

	//	std::cout << "t0 : " << t0 << " t1: " << t1 << std::endl;


		if (t0 > t1) std::swap(t0, t1);
		if (t0 < 0)
		{
			t0 = t1;
			if (t0 < 0)
			{
				std::cout << "t0 < 0" << std::endl;
				return false;
			}
		}

		t = t0;

		return true;

	}

	//get surface data such as normal , texture coordinates type
	void getSurfaceData(const Vec3f& Phit, Vec3f& Nhit, Vec2f& tex) const
	{
		Nhit = Phit - center;
		Nhit.normalize();

		//uv coordinates from sphere normal mapped to [0,1]
		//atan range[-pi , pi] azimuth angle of sphere  gives u
		//acos range[ 0 , pi] polar angle of sphere give v
		tex.x = (1 + atan2(Nhit.z, Nhit.x) / M_PI) * 0.5;
		tex.y = acosf(Nhit.y) / M_PI;
	}

	Vec3f center;
	float radius;
	float radius2;
};

bool trace(const Vec3f& orig, const Vec3f& dir, 
	const std::vector<std::unique_ptr<Object>>& objects, float& tNear, const Object*& hitObject)
{
	tNear = kInfinity;
	std::vector<std::unique_ptr<Object>>::const_iterator iter = objects.begin();
	for (; iter != objects.end(); ++iter)
	{
		float t = kInfinity;
		if ((*iter)->intersect(orig, dir, t) && t < tNear)
		{
			hitObject = iter->get();
			tNear = t;
		}
	}

  return (hitObject != nullptr);
}

Vec3f castRay(const Vec3f& orig, const Vec3f& dir, const std::vector<std::unique_ptr<Object>>& objects)
{
	Vec3f hitColor = 0;
	const Object* hitObject = nullptr;
	float t;

	if (trace(orig, dir, objects, t, hitObject))
	{
		Vec3f Phit = orig + dir * t;
		Vec3f Nhit;
		Vec2f tex;
		hitObject->getSurfaceData(Phit, Nhit, tex);
		float scale = 4;
		float pattern = (fmodf(tex.x * scale, 1) > 0.5 ) ^ (fmodf(tex.y * scale, 1) > 0.5);
		hitColor = std::max(0.f, Nhit.dotProduct(-dir)) *mix(hitObject->color, hitObject->color * 0.8, pattern);
	}

	return hitColor;
}



void render(Options& options , std::vector<std::unique_ptr<Object>> & objects)
{
	Vec3f* framebuffer = new Vec3f[options.width * options.height];
	Vec3f* pix = framebuffer;
	float scale = tan(deg2rad(options.fov * 0.5));
	float imageAspectRatio = options.width / (float)options.height;
	Vec3f orig;
	options.cameraToWorld.multVecMatrix(Vec3f(0), orig);

	for (uint32_t j = 0; j < options.height; ++j)
	{
		for (uint32_t i = 0; i < options.width; ++i)
		{
			float x = (2 * (i + 0.5) / (float)options.width - 1) * imageAspectRatio * scale;
			float y = (1 - 2 * (j + 0.5) / (float)options.height) * scale;
			Vec3f dir;
			options.cameraToWorld.multDirMatrix(Vec3f(x, y, -1), dir);
			dir.normalize();

			*(pix++) = castRay(orig, dir, objects);
		
		}
	}

	//output framebuffer to image
	std::ofstream ofs("./out.ppm", std::ios::out | std::ios::binary);
	assert(ofs);

	ofs << "P6\n" << options.width << " " << options.height << "\n255\n";

	uint32_t N = options.width * options.height;
	std::cerr << "writing " << N << " number of pixels \n";

	for (uint32_t k = 0; k < N; ++k)
	{
		char r = (char)(255 * framebuffer[k].x);
		char g = (char)(255 * framebuffer[k].y);
		char b = (char)(255 * framebuffer[k].z);

		ofs << r << g << b;
	}


	ofs.close();
	
	delete[] framebuffer;
}


int main()
{
	//prepare the scene
	std::vector< std::unique_ptr<Object> > objects;

	uint32_t numSpheres = 32;
	gen.seed(0);
	for (uint32_t i = 0; i < numSpheres; ++i)
	{
		Vec3f randPos((0.5 - dis(gen)) * 10, (0.5 - dis(gen)) * 10, (0.5 + dis(gen) * 10));
		float randRadius = (0.5 + dis(gen) * 0.5);
		objects.push_back(std::unique_ptr<Object>(new Sphere(randPos, randRadius)));	
	}

	Options opt;
	opt.width = 640;
	opt.height = 480;
	opt.fov = 51.52;
	opt.cameraToWorld = Matrix44f(0.945519, 0, -0.325569, 0, -0.179534, 0.834209, -0.521403, 0, 0.271593, 0.551447, 0.78876, 0, 4.208271, 8.374532, 17.932925, 1);

	auto t_start = std::chrono::high_resolution_clock::now();

	render(opt, objects);
	
	auto t_end = std::chrono::high_resolution_clock::now();
	auto passed_time = std::chrono::duration<double, std::milli>(t_end - t_start).count();

	std::cerr << "Render Time: " << passed_time << " ms" << std::endl;

	return 0;
}



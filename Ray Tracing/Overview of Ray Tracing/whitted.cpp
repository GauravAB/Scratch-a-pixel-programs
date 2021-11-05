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
#include <algorithm>


#ifndef M_PI
#define M_PI 3.14159265
#endif

const float kInfinity = std::numeric_limits<float>::max();

class Vec3f
{
public:
	Vec3f() : x(0), y(0), z(0) {}

	Vec3f(float xx) : x(xx), y(xx), z(xx) {}
	Vec3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
	Vec3f operator*(const float& f) const { return Vec3f(x * f + y * f + z * f); }
	Vec3f operator*(const Vec3f& rhs) const { return Vec3f(x * rhs.x , y * rhs.y , z * rhs.z); }
	Vec3f operator-(const Vec3f& v) const { return Vec3f(x - v.x, y - v.y, z - v.z); }
	Vec3f operator+(const Vec3f& v) const { return Vec3f(x + v.x, y + v.y, z + v.z); }
	Vec3f operator-() const { return Vec3f(-x, -y, -z); }
	Vec3f& operator+= (const Vec3f& v) { x += v.x, y += v.y, z += v.z; return *this; }
	friend Vec3f operator*(const float& r, const Vec3f& v)
	{
		return Vec3f(v.x * r, v.y * r, v.z * r);
	}
	friend std::ostream& operator <<(std::ostream& os, const Vec3f& v)
	{
		return os << v.x << ", " << v.y << ", " << v.z;
	}
	float x, y, z;
};


class Vec2f
{
public:
	Vec2f() : x(0), y(0) {}
	Vec2f(float xx) : x(xx), y(xx) {}
	Vec2f(float xx, float yy) : x(xx), y(yy) {}
	Vec2f operator* (const float& r) const { return Vec2f(x * r, y * r); }
	Vec2f operator+(const Vec2f& v) const { return Vec2f(x + v.x, y + v.y); }

	float x, y;
};

Vec3f normalize(const Vec3f& v)
{
	float mag2 = v.x * v.x + v.y * v.y + v.z * v.z;

	if (mag2 > 0)
	{
		float invMag = 1 / sqrtf(mag2);
		Vec3f(v.x * invMag, v.y * invMag, v.z * invMag);
	}

	return v;
}

inline 
float dotProduct(const Vec3f& lhs, const Vec3f& rhs)
{
	return  (lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z);
}

Vec3f crossProduct(const Vec3f& a, const Vec3f& b)
{
	return Vec3f(a.y * b.z - b.y * a.z,
		b.x * a.z - a.x * b.z,
		a.x * b.y - b.x * a.y);
}


inline float clamp(const float& lo, const float& hi, const float& v)
{
	return std::max(lo, std::min(v, hi));
}

inline float deg2rad(const float deg)
{
	return deg * M_PI / 180;
}

inline Vec3f mix(const Vec3f& a, const Vec3f& b, const float& mixValue)
{
	return a * (1 - mixValue) + b * mixValue;
}

struct Options
{
	uint32_t width;
	uint32_t height;
	float fov;
	float imageAspectRatio;
	uint8_t maxDepth;
	Vec3f backgroundColor;
	float bias;
};

class Light
{
public:
	Light(const Vec3f& p, const Vec3f& i) : position(p), intensity(i) {}
	Vec3f position;
	Vec3f intensity;
};

enum MaterialType { DIFFUSE_AND_GLOSSY, REFLECTION_AND_REFRACTION, REFLECTION};

class Object
{
public:
	Object() :
		materialType(DIFFUSE_AND_GLOSSY),
		ior(1.3), Kd(0.8), Ks(0.2), diffuseColor(0.2), specularExponent(25) {}
	virtual ~Object() {}
	virtual bool intersect(const Vec3f&, const Vec3f&, float&, uint32_t&, Vec2f&)const = 0;
	virtual void getSurfaceProperties(const Vec3f&, const Vec3f&, const uint32_t&, const Vec2f&, Vec3f&, Vec2f&) const = 0;
	virtual Vec3f evalDiffuseColor(const Vec2f&) const { return diffuseColor; }

	//material properties
	MaterialType materialType;
	float ior;
	float Kd, Ks;
	Vec3f diffuseColor;
	float specularExponent;
};

bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1)
{
	float discr = b * b - 4 * a * c;
	if (discr < 0) return false;
	else if (discr == 0) { x0 = x1 = -0.5 * b / a; }
	else
	{
		float q = (b > 0) ?
			-0.5 * (b + sqrt(discr)) :
			-0.5 * (b - sqrt(discr));
		x0 = q / a;
		x1 = c / q;
	}
	if (x0 > x1) std::swap(x0, x1);
	return true;
}

class Sphere : public Object
{
public:
	Sphere(const Vec3f& c, const float& r) : center(c), radius(r), radius2(r* r) {}

	bool intersect(const Vec3f& orig, const Vec3f& dir, float& tnear, uint32_t& index, Vec2f& uv) const
	{
		//analytic solution
		Vec3f L = orig - center;
		float a = dotProduct(dir, dir);
		float b = 2 * dotProduct(dir, L);
		float c = dotProduct(L, L) - radius2;
		float t0, t1;
		if (!solveQuadratic(a, b, c, t0, t1)) return false;
		if (t0 < 0) t0 = t1;
		if (t0 < 0) return false;
		tnear = t0;

		return true;
	}

	void getSurfaceProperties(const Vec3f& P, const Vec3f& I, const uint32_t& index, const Vec2f& uv, Vec3f& N, Vec2f& st) const
	{
		N = normalize(P - center);
	}

	Vec3f center;
	float radius;
	float radius2;
};

bool rayTriangleIntersect(
	const Vec3f& v0, const Vec3f& v1, const Vec3f& v2,
	const Vec3f& orig, const Vec3f& dir, float& tnear, float& u, float& v)
{
	Vec3f edge1 = v1 - v0;
	Vec3f edge2 = v2 - v0;
	Vec3f pvec = crossProduct(dir, edge1);
	float det = dotProduct(edge1, pvec);

	if (det == 0 || det < 0) return false;

	Vec3f tvec = orig - v0;
	u = dotProduct(tvec, pvec);
	if (u < 0 || u > det) return false;

	Vec3f qvec = crossProduct(tvec, edge1);
	v = dotProduct(dir, qvec);
	if (v < 0 || u + v > det) return false;

	float invDet = 1 / det;

	tnear = dotProduct(edge2, qvec) * invDet;
	u *= invDet;
	v *= invDet;

	return true;
}



class MeshTriangle : public Object
{
public:
	MeshTriangle(
		const Vec3f* verts,
		const uint32_t* vertsIndex,
		const uint32_t& numTris,
		const Vec2f* st)
	{
		uint32_t maxIndex = 0;
		for (uint32_t i = 0; i < numTris * 3; ++i)
		{
			if (vertsIndex[i] > maxIndex) maxIndex = vertsIndex[i];
		}
		maxIndex += 1;
		vertices = std::unique_ptr<Vec3f[]>(new Vec3f[maxIndex]);
		memcpy(vertices.get(), verts, sizeof(Vec3f) * maxIndex);
		vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);
		memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3);
		numTriangles = numTris;
		stCoordinates = std::unique_ptr<Vec2f[]>(new Vec2f[maxIndex]);
		memcpy(stCoordinates.get(), st, sizeof(Vec2f) * maxIndex);
	}

	
	bool intersect(const Vec3f& orig, const Vec3f& dir, float& tnear, uint32_t& index, Vec2f& uv) const
	{
		bool intersect = false;
		for (uint32_t k = 0; k < numTriangles; ++k)
		{
			const Vec3f& v0 = vertices[vertexIndex[k * 3]];
			const Vec3f& v1 = vertices[vertexIndex[k * 3 + 1]];
			const Vec3f& v2 = vertices[vertexIndex[k * 3 + 2]];

			float t, u, v;
			if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear)
			{
				tnear = t;
				uv.x = u;
				uv.y = v;
				index = k;
				intersect |= true;
			}
		}

		return intersect;
	}

	void getSurfaceProperties(const Vec3f& P, const Vec3f& I, const uint32_t& index, const Vec2f& uv,
		Vec3f& N, Vec2f& st) const
	{
		const Vec3f& v0 = vertices[vertexIndex[index * 3]];
		const Vec3f& v1 = vertices[vertexIndex[index * 3 + 1]];
		const Vec3f& v2 = vertices[vertexIndex[index * 3 + 2]];

		Vec3f e0 = normalize(v1 - v0);
		Vec3f e1 = normalize(v2 - v1);

		N = normalize(crossProduct(e0, e1));
		const Vec2f& st0 = stCoordinates[vertexIndex[index * 3]];
		const Vec2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
		const Vec2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];

		st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
	}

	Vec3f evalDiffuseColor(const Vec2f& st) const
	{
		float scale = 5;
		float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
		return mix(Vec3f(0.815, 0.235, 0.031), Vec3f(0.937, 0.937, 0.231), pattern);
	}

	std::unique_ptr<Vec3f[]> vertices;
	uint32_t numTriangles;
	std::unique_ptr<uint32_t[]> vertexIndex;
	std::unique_ptr<Vec2f[]> stCoordinates;

};

Vec3f reflect(const Vec3f& I, const Vec3f& N)
{
	return I - 2 * dotProduct(I, N) * N;
}

Vec3f refract(const Vec3f& I, const Vec3f& N, const float& ior)
{
	float cosi = clamp(-1, 1, dotProduct(I, N));
	float etai = 1, etat = ior;
	Vec3f n = N;
	if (cosi < 0) { cosi = -cosi; }
	else { std::swap(etai, etat); n = -N; }
	float eta = etai / etat;
	float k = 1 - eta * eta * (1 - cosi * cosi);
	return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
}

void fresnel(const Vec3f& I, const Vec3f& N, const float& ior, float& kr)
{
	float cosi = clamp(-1, 1, dotProduct(I, N));
	float etai = 1, etat = ior;
	if (cosi > 0) { std::swap(etai, etat); }
	
	//compute sini using snell' law
	float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
	//TIR

	if (sint >= 1)
	{
		kr = 1;
	}
	else
	{
		float cost = sqrtf(std::max(0.f, 1 - sint * sint));
		cosi = fabsf(cosi);
		float Rs = ((etai * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
		float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
		kr = (Rs * Rs + Rp * Rp) / 2;
	}

}


bool trace(
	const Vec3f& orig, const Vec3f& dir,
	const std::vector < std::unique_ptr<Object>& objects,
	float& tNear, uint32_t& index, Vec2f& uv, Object** hitObject)
{

	*hitObject = nullptr;
	for (uint32_t k = 0; k < objects.size(); ++k)
	{
		float tNearK = kInfinity;
		uint32_t indexK;
		Vec2f uvK;

		if (objects[k]->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear)
		{
			*hitObject = objects[k].get();
			tNear = tNearK;
			index = indexK;
			uv = uvK;
		}
	}
	

	return (*hitObject != nullptr);
}
Vec3f castRay(
	const Vec3f& orig, const Vec3f& dir,
	const std::vector<std::unique_ptr<Object>>& objects,
	const std::vector<std::unique_ptr<Light>>& lights,
	const Options& options,
	uint32_t depth,
	bool test = false
)
{
	if (depth > options.maxDepth)
	{
		return options.backgroundColor;
	}

	Vec3f hitColor = options.backgroundColor;
	float tnear = kInfinity;
	Vec2f uv;
	uint32_t index = 0;
	Object* hitObject = nullptr;
	if (trace(orig, dir, objects, tnear, index, uv, &hitObject))
	{
		Vec3f hitPoint = orig + dir * tnear;
		Vec3f N;
		Vec2f st;
		hitObject->getSurfaceProperties(hitPoint, dir, index, uv, N, st);
		Vec3f tmp = hitPoint;
		switch (hitObject->materialType)
		{
		case REFLECTION_AND_REFRACTION:
			Vec3f refectionDirection = normalize(reflect(dir, N));
			Vec3f refractionDirection = normalize(refract(dir, N, hitObject->ior));
			Vec3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
				hitPoint - N * options.bias :
				hitPoint + N * options.bias;
			Vec3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
				hitPoint - N * options.bias :
				hitPoint + N * options.bias;
			Vec3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, objects, lights, options, depth + 1, 1);
			Vec3f refractionColor = castRay(refractionRayOrig, refractionDirection, objects, lights, options, depth + 1, 1);
			float kr;
			fresnel(dir, N, hitObject->ior, kr);
			hitColor = reflectionColor * kr + refractionColor * (1 - kr);
			break;
		case REFLECTION:
			float kr;
			fresnel(dir, N, hitObject->ior, kr);
			Vec3f reflectionDirection = reflect(dir, N);
			Vec3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
				hitPoint - N * options.bias :
				hitPoint + N * options.bias;
			hitColor = castRay(reflectionRayOrig, reflectionDirection, objects, lights, options, depth + 1);
			break;
		default:
			Vec3f lightAmt = 0, specularColor = 0;
			Vec3f shadowPointOrig = (dotProduct(dir, N) < 0) ?
				hitPoint + N * options.bias :
				hitPoint - N * options.bias;
			for (uint32_t i = 0; i < lights.size(); ++i)
			{
				Vec3f lightDir = lights[i]->position - hitPoint;
				float lightDistance2 = dotProduct(lightDir, lightDir);
				lightDir = normalize(lightDir);
				float LdotN = std::max(0.f, dotProduct(lightDir, N));
				Object* shadowHitObject = nullptr;
				float tNearShadow = kInfinity;
				bool inShadow = trace(shadowPointOrig, lightDir, objects, tNearShadow, index, uv, &shadowHitObject) &&
					tNearShadow * tNearShadow < lightDistance2;
				lightAmt += (1 - inShadow) * lights[i]->intensity * LdotN;
				Vec3f reflectionDirection = reflect(-lightDir, N);
				specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)), hitObject->specularExponent) * lights[i]->intensity;
			}
			hitColor = lightAmt * hitObject->evalDiffuseColor(st) * hitObject->Kd + specularColor * hitObject->Ks;
			break;
		}
	}

	return hitColor;
}



int main() {

	return 0;
}


































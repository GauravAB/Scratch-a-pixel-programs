
#define _USE_MATH_DEFINES

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <utility>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>
#include <cassert>


#include "geometry.h"
#include "model.h"


static const float kInfinity = std::numeric_limits<float>::max();
static const float kEpsilon = 1e-8;
static const Vec3f kDefaultBackgroundColor = Vec3f(0.235294, 0.67451, 0.843137);

inline float clamp(const float& lo, const float& hi, const float& v)
{
	return std::max(lo, std::min(hi, v));
}
inline float deg2rad(const float& d)
{
	return  d * M_PI / 180;
}

struct Options
{
	uint32_t width = 640;
	uint32_t height = 480;
	float fov = 90;
	Vec3f backgroundColor = kDefaultBackgroundColor;
	Matrix44f cameraToWorld;
};

class Object
{
public:
	Object() {}
	virtual ~Object() {}
	virtual bool intersect(const Vec3f&, const Vec3f&, float&, uint32_t&, Vec2f&) const = 0;
	virtual void getSurfaceProperties(const Vec3f&, const Vec3f&, const uint32_t&, const Vec2f&, Vec3f&, Vec2f&) const = 0;
	virtual Vec2f getBBoxMin() { return Vec2f(0); }
	virtual Vec2f getBBoxMax() { return Vec2f(0); }

};

bool rayTriangleIntersect(
	const Vec3f& orig,
	const Vec3f& dir,
	const Vec3f& v0,const Vec3f& v1, const Vec3f& v2,
	float &t, float &u, float &v
)
{
	Vec3f v0v1 = v1 - v0;
	Vec3f v0v2 = v2 - v0;
	Vec3f pvec = dir.crossProduct(v0v2);
	float det = v0v1.dotProduct(pvec);

	//ray and triangle are parallel if det is close to 0
	
	
	if(fabs(det) < kEpsilon) { return false;}
	
	float invDet = 1 / det;

	//calculate distance v0 to ray origin
	Vec3f tvec = orig - v0;

	u = tvec.dotProduct(pvec) * invDet;
	if (u < 0 || u  > 1) return false;

	Vec3f qvec = tvec.crossProduct(v0v1);
	v = dir.dotProduct(qvec) * invDet;
	if ((v < 0) || (u + v) > 1) return false;

	t = v0v2.dotProduct(qvec) * invDet;

/*
if( (u + v + (1.0 - (u + v))) != 1.0 )
	{
		std::cout << "u v w " << u << " " << v << " " << (1.0 - u - v) <<   " = " << (float)(u + v + (1.0 - (u + v))) << std::endl;
	}
*/

	return true;
}

class TriangleMesh : public Object
{
public:
	//Build triangle from face index and vertex index array
	TriangleMesh(
		const uint32_t nfaces,
		const std::unique_ptr<uint32_t[] >& faceIndex,
		const std::unique_ptr<uint32_t[] >& vertsIndex,
		const std::unique_ptr<Vec3f[]>& verts,
		std::unique_ptr<Vec3f[]>& normals,
		std::unique_ptr<Vec2f[]>& st) : numTris(0),BBoxMin(kInfinity),BBoxMax(-kInfinity)
	{
		
		uint32_t k = 0, maxVertIndex = 0;
		//find out how many triangles we need to create for this mesh
		for (uint32_t i = 0; i < nfaces; ++i)
		{
			numTris += faceIndex[i] - 2;
			for (uint32_t j = 0; j < faceIndex[i]; ++j)
			{
				if (vertsIndex[k + j] > maxVertIndex)
					maxVertIndex = vertsIndex[k + j];
			}
			k += faceIndex[i];
		}

		maxVertIndex += 1;

		//allocate memory to store the positions of the mesh vertices
		P = std::unique_ptr<Vec3f[]>(new Vec3f[maxVertIndex]);
		for (uint32_t i = 0; i < maxVertIndex; ++i)
		{
			P[i] = verts[i];
		}

		//allocate memory to store triangle indices
		trisIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);
		uint32_t l = 0;

		N = std::unique_ptr < Vec3f[]>(new Vec3f[numTris * 3]);
		texCoordinates = std::unique_ptr<Vec2f[]>(new Vec2f[numTris * 3]);
		
		for (uint32_t i = 0, k = 0; i < nfaces; ++i) //for each face
		{
			for (uint32_t j = 0; j < faceIndex[i] - 2; ++j) //for each triangle in the face
			{
				//triangulate the quad
				trisIndex[l] = vertsIndex[k];
				trisIndex[l + 1] = vertsIndex[k + j + 1];
				trisIndex[l + 2] = vertsIndex[k + j + 2];
				N[l] = normals[k];
				N[l + 1] = normals[k + j + 1];
				N[l + 2] = normals[k + j + 2];
				texCoordinates[l] = st[k];
				texCoordinates[l + 1] = st[k + j + 1];
				texCoordinates[l + 2] = st[k + j + 2];
				l += 3;
			}
			k += faceIndex[i];
		}

		//calculate bbox for this mesh
		for (uint32_t i = 0; i < numTris * 3; ++i)
		{
				if (BBoxMax.x < verts[i].x) BBoxMax.x = verts[i].x;
				if (BBoxMax.y < verts[i].y) BBoxMax.y = verts[i].y;
			
				if (BBoxMin.x > verts[i].x) BBoxMin.x = verts[i].x;
				if (BBoxMin.y > verts[i].y) BBoxMin.y = verts[i].y;
		}



	}

	bool intersect(const Vec3f& orig, const Vec3f& dir, float& tNear, uint32_t& triIndex, Vec2f& uv) const
	{
		uint32_t j = 0;
		bool isect = false;
		for (uint32_t i = 0; i < numTris; ++i)
		{
			const Vec3f& v0 = P[trisIndex[j]];
			const Vec3f& v1 = P[trisIndex[j + 1]];
			const Vec3f& v2 = P[trisIndex[j + 2]];

			float t = kInfinity, u, v;
			if (rayTriangleIntersect(orig, dir, v0, v1, v2, t, u, v) && t < tNear)
			{
				tNear = t;
				uv.x = u;
				uv.y = v;
				triIndex = i;
				isect = true;
			}
			j += 3;
		}

		return isect;
	}

	void getSurfaceProperties(
		const Vec3f& hitPoint,
		const Vec3f& viewDirection,
		const uint32_t& triIndex,
		const Vec2f& uv,
		Vec3f& hitNormal,
		Vec2f& hitTextureCoordinates) const
	{
		//face normal
		const Vec3f& v0 = P[trisIndex[triIndex * 3]];
		const Vec3f& v1 = P[trisIndex[triIndex * 3 + 1]];
		const Vec3f& v2 = P[trisIndex[triIndex * 3 + 2]];
		hitNormal = (v1 - v0).crossProduct(v2 - v0);
		hitNormal.normalize();

		//texture coordinates
		const Vec2f& st0 = texCoordinates[triIndex * 3];
		const Vec2f& st1 = texCoordinates[triIndex * 3 + 1];
		const Vec2f& st2 = texCoordinates[triIndex * 3 + 2];
		hitTextureCoordinates = (1 - uv.x - uv.y) * st0 + uv.x * st1 + uv.y * st2;

		//vertex normals
		const Vec3f& n0 = N[triIndex * 3];
		const Vec3f& n1 = N[triIndex * 3+ 1];
		const Vec3f& n2 = N[triIndex * 3+ 2];
		hitNormal = (1 - uv.x - uv.y) * n0 + uv.x * n1 + uv.y * n2;
	
	}

	Vec2f getBBoxMin() { return BBoxMin; }
	Vec2f getBBoxMax() { return BBoxMax; }

	uint32_t numTris;
	std::unique_ptr<Vec3f[]> P;
	std::unique_ptr<uint32_t[]> trisIndex;
	std::unique_ptr<Vec3f[]> N;
	std::unique_ptr<Vec2f[]> texCoordinates;
	Vec2f BBoxMin;
	Vec2f BBoxMax;

};

class ObjMesh : public Object
{
public:
	ObjMesh(const char* file)
	{
		if (!loadObjFile(file))
		{
			std::cout << "Obj Mesh failed\n";
		}
	}


	bool loadObjFile(const char* file);



	bool intersect(const Vec3f& orig, const Vec3f& dir, float& tNear, uint32_t& triIndex, Vec2f& uv) const
	{
		bool isect = false;
		uint32_t numTris = model->nfaces();

		for (uint32_t i = 0; i < numTris; ++i)
		{
			std::vector<int> face = model->face(i);
			const Vec3f& v0 =  model->vert(face[0]);
			const Vec3f& v1 = model->vert(face[3]);
			const Vec3f& v2 = model->vert(face[6]);

			float t = kInfinity, u, v;
			if (rayTriangleIntersect(orig, dir, v0, v1, v2, t, u, v) && t < tNear)
			{
				tNear = t;
				uv.x = u;
				uv.y = v;
				triIndex = i;
				isect = true;
			}
		}

		return isect;
	}

	
	void getSurfaceProperties(
		const Vec3f& hitPoint,
		const Vec3f& viewDirection,
		const uint32_t& triIndex,
		const Vec2f& uv,
		Vec3f& hitNormal,
		Vec2f& hitTextureCoordinates) const
	{
		//face normal for flat shading
		std::vector<int> face = model->face(triIndex);

		/*
 		const Vec3f& v0 = model->vert(face[0]);
		const Vec3f& v1 = model->vert(face[3]);
		const Vec3f& v2 = model->vert(face[6]);
		hitNormal = (v2 - v0).crossProduct(v1 - v0);
		hitNormal.normalize();
		*/

		//texture coordinates
		const Vec2f & st0 = model->st(face[1]);
		const Vec2f & st1 = model->st(face[4]);
		const Vec2f & st2 = model->st(face[7]);
		hitTextureCoordinates = (1.0f - uv.x - uv.y) * st0 + uv.x * st1 + uv.y * st2;

		//vertex normals for smooth shading
		Vec3f n0 = model->norm(face[2]);
		n0.normalize();
		Vec3f  n1 = model->norm(face[5]);
		n1.normalize();
		Vec3f  n2 = model->norm(face[8]);
		n2.normalize();
		hitNormal = (1 - (uv.x + uv.y) ) * n0 + uv.x * n1 + uv.y * n2;
		
	}


	std::shared_ptr<Model> model;
	uint32_t numTris;
};


bool ObjMesh::loadObjFile(const char* file)
{
	model = std::make_shared<Model>(file);

	return true;
}


/*
TriangleMesh* generatePolySphere(float rad, uint32_t divs)
{
	//generate points
	uint32_t numVertices = (divs - 1) * divs + 2;
	std::unique_ptr<Vec3f[]> P(new Vec3f[numVertices]);
	std::unique_ptr<Vec3f[]> N(new Vec3f[numVertices]);
	std::unique_ptr<Vec2f[]> st(new Vec2f[numVertices]);

	//phi going from -pi/2 to pi/2
	float u = -M_PI_2;
	//theta going from -pi to pi 
	float v = -M_PI;
	float du = M_PI / divs;
	float dv = 2 * M_PI / divs;

	P[0] = N[0] = Vec3f(0, -rad, 0);
	uint32_t k = 1;

	for (uint32_t i = 0; i < divs - 1; i++)
	{
		u += du;
		v = -M_PI;

		for (uint32_t j = 0; j < divs; j++)
		{
			float x = rad * cos(u) * cos(v);
			float y = rad * rad * sin(u);
			float z = rad * cos(u) * sin(v);

			P[k] = N[k] = Vec3f(x, y, z);
			st[k].x = u / M_PI + 0.5;
			st[k].y = v * 0.5 / M_PI + 0.5;
			v += dv, k++;
		}
	}

	P[k] = N[k] = Vec3f(0, rad, 0);
	uint32_t npolys = divs * divs;
	std::unique_ptr<uint32_t[]> faceIndex(new uint32_t[npolys]);
	std::unique_ptr<uint32_t[]> vertsIndex(new uint32_t[(6 + (divs - 1) * 4) * divs]);

	//create the connectivity lists
	uint32_t vid = 1, numV = 0, l = 0;
	k = 0;
	for (uint32_t i = 0; i < divs; i++)
	{
		for (uint32_t j = 0; j < divs; j++)
		{
			if (i == 0)
			{
				faceIndex[k++] = 3;
				vertsIndex[l] = 0;
				vertsIndex[l + 1] = j + vid;
				vertsIndex[l + 2] = (j == (divs - 1)) ? vid : j + vid + 1;
				l += 3;
			}
			else if (i == (divs - 1))
			{
				faceIndex[k++] = 3;
				vertsIndex[l] = j + vid + 1 - divs;
				vertsIndex[l + 1] = vid + 1;
				vertsIndex[l + 2] = (j == (divs - 1)) ? vid + 1 - divs : j + vid + 2 - divs;
				l += 3;
			}
			else
			{
				faceIndex[k++] = 4;
				vertsIndex[l] = j + vid + 1 - divs;
				vertsIndex[l + 1] = j + vid + 1;
				vertsIndex[l + 2] = (j == (divs - 1)) ? vid + 1 : j + vid + 2;
				vertsIndex[l + 3] = (j == (divs - 1)) ? vid + 1 - divs : j + vid + 2 - divs;
				l += 4;
			}
			numV++;
		}
		vid = numV;
	}


	return new TriangleMesh(npolys, faceIndex, vertsIndex, P, N, st);
}
*/
TriangleMesh* generatePolySphere(float rad, uint32_t divs) 
{ 
    // generate points                                                                                                                                                                                      
    uint32_t numVertices = (divs - 1) * divs + 2; 
    std::unique_ptr<Vec3f []> P(new Vec3f[numVertices]); 
    std::unique_ptr<Vec3f []> N(new Vec3f[numVertices]); 
    std::unique_ptr<Vec2f []> st(new Vec2f[numVertices]); 
 
    float u = -M_PI_2; 
    float v = -M_PI; 
    float du = M_PI / divs; 
    float dv = 2 * M_PI / divs; 
 
    P[0] = N[0] = Vec3f(0, -rad, 0); 
    uint32_t k = 1; 
    for (uint32_t i = 0; i < divs - 1; i++) { 
        u += du; 
        v = -M_PI; 
        for (uint32_t j = 0; j < divs; j++) { 
            float x = rad * cos(u) * cos(v); 
            float y = rad * sin(u); 
            float z = rad * cos(u) * sin(v) ; 
            P[k] = N[k] = Vec3f(x, y, z); 
            st[k].x = u / M_PI + 0.5; 
            st[k].y = v * 0.5 / M_PI + 0.5; 
            v += dv, k++; 
        } 
    } 
    P[k] = N[k] = Vec3f(0, rad, 0); 
 
    uint32_t npolys = divs * divs; 
    std::unique_ptr<uint32_t []> faceIndex(new uint32_t[npolys]); 
    std::unique_ptr<uint32_t []> vertsIndex(new uint32_t[(6 + (divs - 1) * 4) * divs]); 
 
    // create the connectivity lists                                                                                                                                                                        
    uint32_t vid = 1, numV = 0, l = 0; 
    k = 0; 
    for (uint32_t i = 0; i < divs; i++) { 
        for (uint32_t j = 0; j < divs; j++) { 
            if (i == 0) { 
                faceIndex[k++] = 3; 
                vertsIndex[l] = 0; 
                vertsIndex[l + 1] = j + vid; 
                vertsIndex[l + 2] = (j == (divs - 1)) ? vid : j + vid + 1; 
                l += 3; 
            } 
            else if (i == (divs - 1)) { 
                faceIndex[k++] = 3; 
                vertsIndex[l] = j + vid + 1 - divs; 
                vertsIndex[l + 1] = vid + 1; 
                vertsIndex[l + 2] = (j == (divs - 1)) ? vid + 1 - divs : j + vid + 2 - divs; 
                l += 3; 
            } 
            else { 
                faceIndex[k++] = 4; 
                vertsIndex[l] = j + vid + 1 - divs; 
                vertsIndex[l + 1] = j + vid + 1; 
                vertsIndex[l + 2] = (j == (divs - 1)) ? vid + 1 : j + vid + 2; 
                vertsIndex[l + 3] = (j == (divs - 1)) ? vid + 1 - divs : j + vid + 2 - divs; 
                l += 4; 
            } 
            numV++; 
        } 
        vid = numV; 
    } 
 
    return new TriangleMesh(npolys, faceIndex, vertsIndex, P, N, st); 
} 


TriangleMesh* loadPolyMeshFromFile(const char* file)
{
	std::ifstream ifs;
	try
	{
		ifs.open(file);
		if (ifs.fail()) throw;
		std::stringstream ss;
		ss << ifs.rdbuf();
		uint32_t numFaces;
		ss >> numFaces;
		std::unique_ptr<uint32_t[]> faceIndex(new uint32_t[numFaces]);
		uint32_t vertsIndexArraySize = 0;
		//reading face index
		for (uint32_t i = 0; i < numFaces; ++i)
		{
			ss >> faceIndex[i];
			vertsIndexArraySize += faceIndex[i];
		}
		std::unique_ptr<uint32_t[]> vertsIndex(new uint32_t[vertsIndexArraySize]);
		uint32_t vertsArraySize = 0;
		//reading vertex index array
		for (uint32_t i = 0; i < vertsIndexArraySize; i++)
		{
			ss >> vertsIndex[i];
			if (vertsIndex[i] > vertsArraySize) vertsArraySize = vertsIndex[i];
		}
		vertsArraySize += 1;		
		//reading vertices
		std::unique_ptr<Vec3f[]> verts(new Vec3f[vertsArraySize]);
		for (uint32_t i = 0; i < vertsArraySize; ++i)
		{
			ss >> verts[i].x >> verts[i].y >> verts[i].z;
		}

		//reading normals
		std::unique_ptr<Vec3f[]> normals(new Vec3f[vertsIndexArraySize]);
		for (uint32_t i = 0; i < vertsIndexArraySize; ++i)
		{
			ss >> normals[i].x >> normals[i].y >> normals[i].z;
		}
		//reading st coordinates
		std::unique_ptr<Vec2f[]> st(new Vec2f[vertsIndexArraySize]);
		for (uint32_t i = 0; i < vertsIndexArraySize; ++i)
		{
			ss >> st[i].x >> st[i].y;
		}

		std::cout << "file " << file << " loaded: ";
		std::cout << " numfaces : " << numFaces;
		std::cout << " verts: " << vertsIndexArraySize << "\n";
	
		return new TriangleMesh(numFaces, faceIndex, vertsIndex, verts, normals, st);
	}

	catch (...)
	{
		ifs.close();
	}

	ifs.close();

	return nullptr;
}



bool trace(
	const Vec3f& orig, const Vec3f& dir,
	const std::vector<std::unique_ptr<Object>>& objects,
	float & tNear, uint32_t& index, Vec2f& uv, Object** hitObject)
{
	*hitObject = nullptr;
	for (uint32_t k = 0; k < objects.size(); ++k)
	{
		float tNearTriangle = kInfinity;
		uint32_t indexTriangle;
		Vec2f uvTriangle;

		if (objects[k]->intersect(orig, dir, tNearTriangle, indexTriangle, uvTriangle) && tNearTriangle < tNear)
		{
			tNear = tNearTriangle;
			*hitObject = objects[k].get();
			index = indexTriangle;
			uv = uvTriangle;
		}
	}

	return (*hitObject != nullptr);
}

Vec3f castRay(const Vec3f& orig, const Vec3f& dir,
	const std::vector<std::unique_ptr<Object>>& objects,
	const Options& options)
{
	Vec3f hitColor = options.backgroundColor * dir.y + (1 - dir.y) * Vec3f(1.0,1.0,1.0);
	float tnear = kInfinity;
	Vec2f uv;
	uint32_t index = 0;
	Object* hitObject = nullptr;
	Vec3f lightDir = Vec3f(0,0,-1);

	if (trace(orig, dir, objects, tnear, index, uv, &hitObject))
	{
		Vec3f hitPoint = orig + dir * tnear;
		Vec3f hitNormal;
		Vec2f hitTexCoordinates;
		hitObject->getSurfaceProperties(hitPoint, dir, index, uv, hitNormal, hitTexCoordinates);
		float NdotView = std::max(0.f, hitNormal.dotProduct(-dir));
		float NdotL = std::max(0.f,hitNormal.dotProduct(lightDir));
		const int M = 10;
		float checker = (fmod(hitTexCoordinates.x * M, 1.0) > 0.5) ^ (fmod(hitTexCoordinates.y * M, 1.0) < 0.5);
		float c = 0.3 * (1 - checker) + 0.7 * checker;

		hitColor = c * NdotView;
	//	hitColor =  Vec3f(1)*NdotL;
	}

	return hitColor;
}



void render(
	const Options& options,
	const std::vector<std::unique_ptr<Object>>& objects,
	const uint32_t& frame)
{	
	const float imageAspectRatio = options.width / (float)options.height;

	std::unique_ptr<Vec3f[]> framebuffer(new Vec3f[options.width * options.height]);
	Vec3f* pix = framebuffer.get();
	float scale = tan(deg2rad(options.fov * 0.5));

	Vec3f orig;
	options.cameraToWorld.multVecMatrix(Vec3f(0), orig);

	auto timeStart = std::chrono::high_resolution_clock::now();

	for (uint32_t j = 0; j < options.height; ++j)
	{
		for (uint32_t i = 0; i < options.width; ++i)
		{
			float x = (2 * (i + 0.5) / (float)options.width - 1) * imageAspectRatio * scale;
			float y = (1 - 2 * (j + 0.5) / (float)options.height) * scale;
			Vec3f dir;
			options.cameraToWorld.multDirMatrix(Vec3f(x, y, -1), dir);
			dir.normalize();
			*(pix++) = castRay(orig, dir, objects, options);
		}
	}


	auto timeEnd = std::chrono::high_resolution_clock::now();
	auto passedTime = std::chrono::duration<float, std::milli>(timeEnd - timeStart).count();
	fprintf(stderr, "\rDone: %.2f (sec)\n", passedTime / 1000);

	//save famrbuffer to file
	char buff[256];
	//sprintf(buff, "out.%04d.ppm", frame);
	sprintf(buff,"torus.%04d.ppm",frame);
	std::ofstream ofs;
	ofs.open(buff,std::ios::out | std::ios::binary);
	ofs << "P6\n" << options.width << " " << options.height << "\n255\n";
	for (uint32_t i = 0; i < options.height * options.width; ++i)
	{
		char r = (char)(255 * clamp(0, 1, framebuffer[i].x));
		char g = (char)(255 * clamp(0, 1, framebuffer[i].y));
		char b = (char)(255 * clamp(0, 1, framebuffer[i].z));
		ofs << r << g << b;
	}

	ofs.close();
}


int main()
{
	//setting options
	Options options;
  //  Matrix44f tmp = Matrix44f(0.707107, -0.331295, 0.624695, 0, 0, 0.883452, 0.468521, 0, -0.707107, -0.331295, 0.624695, 0, -1.63871, -5.747777, -40.400412, 1);
   // options.cameraToWorld = tmp.inverse();
	options.fov = 110.0;



	std::vector<std::unique_ptr<Object>> objects;

	//using triangle mesh
	/*
    TriangleMesh* mesh = loadPolyMeshFromFile("teapot.geo");
	if (mesh)
	{
		objects.push_back(std::unique_ptr<Object>(mesh));
	}
*/

	objects.push_back(std::unique_ptr<Object>(new ObjMesh("torus.obj")));
	render(options, objects, 0);
	
/*
	for(uint32_t i = 0; i < 10; ++i)
	{
		int divs = 5 + i;
		//Creating the scene 
		std::vector<std::unique_ptr<Object>> objects;
		TriangleMesh* mesh = generatePolySphere(2,divs);
		objects.push_back(std::unique_ptr<Object>(mesh));

		render(options , objects , i);
	}
*/
	return 0;
}








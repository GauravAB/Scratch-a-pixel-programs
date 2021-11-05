#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>
#include <sstream>
#include <vector>
#include <random>


#include "geometry.h"

const uint32_t w = 640;
const uint32_t h = 480;
const float kEpsilon = 1e-8;
const float kInfinity = std::numeric_limits<float>::max();
const Vec3f kDefaultBackgroundColor = Vec3f(0.235294,0.67451,0.843137);
constexpr float oneOverGamma = 1.f;

struct Options
{
    uint32_t width = w;
    uint32_t height = h;
    float fov = 90;
    Vec3f bgColor = kDefaultBackgroundColor;
    Matrix44f cameraToWorld;
    float bias = 0.0001;
    uint32_t maxDepth = 2;
};

template<typename T>
T clamp(const T& lo, const T& hi, const T& v)
{
    return std::max(lo , std::min(hi,v));
}

template<typename T>
T mix(const T& A , const T& B, float t)
{
    return (A * t + B * (1.0 - t ));
}

float degToRad( float deg)
{
    return ( deg * M_PI / 180);
}

enum MaterialType
{
    kDiffuse,
};

class Object
{
    public:
    Object(const Matrix44f &o2w) : objectToWorld(o2w), worldToObject(o2w.inverse()){}
    virtual ~Object() {}
    virtual bool intersect(const Vec3f& , const Vec3f& , float&, uint32_t &, Vec2f&) const = 0;
    virtual void getSurfaceProperties(const Vec3f& , const Vec3f&, const uint32_t&, const Vec2f&, Vec3f&, Vec2f&) const  =0;

    Matrix44f objectToWorld, worldToObject;
    MaterialType type = kDiffuse;
    Vec3f albedo = 0.18;

    float Kd = 0.8;
    float Ks = 0.2;
    float n = 10;

};

bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0) return false;
    else if (discr == 0) {
        x0 = x1 = - 0.5 * b / a;
    }
    else {
        float q = (b > 0) ?
            -0.5 * (b + sqrt(discr)) :
            -0.5 * (b - sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }

    return true;
}

// [comment]
// Sphere class. A sphere type object
// [/comment]
class Sphere : public Object
{
public:
    Sphere(const Matrix44f &o2w, const float &r) : Object(o2w), radius(r), radius2(r *r)
    { o2w.multVecMatrix(Vec3f(0), center); }
    // [comment]
    // Ray-sphere intersection test
    // [/comment]
    bool intersect(
        const Vec3f &orig,
        const Vec3f &dir,
        float &tNear,
        uint32_t &triIndex, // not used for sphere
        Vec2f &uv) const    // not used for sphere
    {
        float t0, t1; // solutions for t if the ray intersects
        // analytic solution
        Vec3f L = orig - center;
        float a = dir.dotProduct(dir);
        float b = 2 * dir.dotProduct(L);
        float c = L.dotProduct(L) - radius2;
        if (!solveQuadratic(a, b, c, t0, t1)) return false;

        if (t0 > t1) std::swap(t0, t1);

        if (t0 < 0) {
            t0 = t1; // if t0 is negative, let's use t1 instead
            if (t0 < 0) return false; // both t0 and t1 are negative
        }

        tNear = t0;

        return true;
    }
    // [comment]
    // Set surface data such as normal and texture coordinates at a given point on the surface
    // [/comment]
    void getSurfaceProperties(
        const Vec3f &hitPoint,
        const Vec3f &viewDirection,
        const uint32_t &triIndex,
        const Vec2f &uv,
        Vec3f &hitNormal,
        Vec2f &hitTextureCoordinates) const
    {
        hitNormal = hitPoint - center;
        hitNormal.normalize();
        // In this particular case, the normal is simular to a point on a unit sphere
        // centred around the origin. We can thus use the normal coordinates to compute
        // the spherical coordinates of Phit.
        // atan2 returns a value in the range [-pi, pi] and we need to remap it to range [0, 1]
        // acosf returns a value in the range [0, pi] and we also need to remap it to the range [0, 1]
        hitTextureCoordinates.x = (1 + atan2(hitNormal.z, hitNormal.x) / M_PI) * 0.5;
        hitTextureCoordinates.y = acosf(hitNormal.y) / M_PI;
    }
    float radius, radius2;
    Vec3f center;
};


bool rayTriangleIntersect(
    const Vec3f &orig, const Vec3f& dir,
    const Vec3f &v0, const Vec3f& v1, const Vec3f& v2,
    float &t, float &u, float &v)
    {
        Vec3f v0v1 = v1 - v0;
        Vec3f v0v2 = v2 - v0;

        Vec3f pvec = dir.crossProduct(v0v2);
        float det = v0v1.dotProduct(pvec);

        if(fabs(det) < kEpsilon) return false;

        float invDet = 1 / det;

        Vec3f tvec = orig - v0;
        u = tvec.dotProduct(pvec) * invDet;
        if( u < 0 || u > 1) return false;

        Vec3f qvec = tvec.crossProduct(v0v1);
        v = dir.dotProduct(qvec) * invDet;
        if(v < 0 || u + v > 1) return false;

        t = v0v2.dotProduct(qvec) * invDet;
        
        return (t > 0) ? true : false;

    }

class TriangleMesh : public Object
{
public:
    // Build a triangle mesh from a face index array and a vertex index array
    TriangleMesh(
        const Matrix44f &o2w,
        const uint32_t nfaces,
        const std::unique_ptr<uint32_t []> &faceIndex,
        const std::unique_ptr<uint32_t []> &vertsIndex,
        const std::unique_ptr<Vec3f []> &verts,
        std::unique_ptr<Vec3f []> &normals,
        std::unique_ptr<Vec2f []> &st) :
        Object(o2w),
        numTris(0)
    {
        uint32_t k = 0, maxVertIndex = 0;
        // find out how many triangles we need to create for this mesh
        for (uint32_t i = 0; i < nfaces; ++i) {
            numTris += faceIndex[i] - 2;
            for (uint32_t j = 0; j < faceIndex[i]; ++j)
                if (vertsIndex[k + j] > maxVertIndex)
                    maxVertIndex = vertsIndex[k + j];
            k += faceIndex[i];
        }
        maxVertIndex += 1;
        
        // allocate memory to store the position of the mesh vertices
        P = std::unique_ptr<Vec3f []>(new Vec3f[maxVertIndex]);
        for (uint32_t i = 0; i < maxVertIndex; ++i) {
            // [comment]
            // Transforming vertices to world space
            // [/comment]
            objectToWorld.multVecMatrix(verts[i], P[i]);
        }
        
        // allocate memory to store triangle indices
        trisIndex = std::unique_ptr<uint32_t []>(new uint32_t [numTris * 3]);
        uint32_t l = 0;
        N = std::unique_ptr<Vec3f []>(new Vec3f[numTris * 3]);
        sts = std::unique_ptr<Vec2f []>(new Vec2f[numTris * 3]);
        // [comment]
        // Computing the transpse of the object-to-world inverse matrix
        // [/comment]
        Matrix44f transformNormals = worldToObject.transpose();
        // generate the triangle index array and set normals and st coordinates
        for (uint32_t i = 0, k = 0; i < nfaces; ++i) { // for each  face
            for (uint32_t j = 0; j < faceIndex[i] - 2; ++j) { // for each triangle in the face
                trisIndex[l] = vertsIndex[k];
                trisIndex[l + 1] = vertsIndex[k + j + 1];
                trisIndex[l + 2] = vertsIndex[k + j + 2];
                // [comment]
                // Transforming normals
                // [/comment]
                transformNormals.multDirMatrix(normals[k], N[l]);
                transformNormals.multDirMatrix(normals[k + j + 1], N[l + 1]);
                transformNormals.multDirMatrix(normals[k + j + 2], N[l + 2]);
                N[l].normalize();
                N[l + 1].normalize();
                N[l + 2].normalize();
                sts[l] = st[k];
                sts[l + 1] = st[k + j + 1];
                sts[l + 2] = st[k + j + 2];
                l += 3;
            }                                                                                                                                                                                                                                
            k += faceIndex[i];
        }
    }
    // Test if the ray interesests this triangle mesh
    bool intersect(const Vec3f &orig, const Vec3f &dir, float &tNear, uint32_t &triIndex, Vec2f &uv) const
    {
        uint32_t j = 0;
        bool isect = false;
        for (uint32_t i = 0; i < numTris; ++i) {
            const Vec3f &v0 = P[trisIndex[j]];
            const Vec3f &v1 = P[trisIndex[j + 1]];
            const Vec3f &v2 = P[trisIndex[j + 2]];
            float t = kInfinity, u, v;
            if (rayTriangleIntersect(orig, dir, v0, v1, v2, t, u, v) && t < tNear) {
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
        const Vec3f &hitPoint,
        const Vec3f &viewDirection,
        const uint32_t &triIndex,
        const Vec2f &uv,
        Vec3f &hitNormal,
        Vec2f &hitTextureCoordinates) const
    {
        if (smoothShading) {
            // vertex normal
            const Vec3f &n0 = N[triIndex * 3];
            const Vec3f &n1 = N[triIndex * 3 + 1];
            const Vec3f &n2 = N[triIndex * 3 + 2];
            hitNormal = (1 - uv.x - uv.y) * n0 + uv.x * n1 + uv.y * n2;
        }
        else {
            // face normal
            const Vec3f &v0 = P[trisIndex[triIndex * 3]];
            const Vec3f &v1 = P[trisIndex[triIndex * 3 + 1]];
            const Vec3f &v2 = P[trisIndex[triIndex * 3 + 2]];
            hitNormal = (v1 - v0).crossProduct(v2 - v0);
        }

        // doesn't need to be normalized as the N's are normalized but just for safety
        hitNormal.normalize();

        // texture coordinates
        const Vec2f &st0 = sts[triIndex * 3];
        const Vec2f &st1 = sts[triIndex * 3 + 1];
        const Vec2f &st2 = sts[triIndex * 3 + 2];
        hitTextureCoordinates = (1 - uv.x - uv.y) * st0 + uv.x * st1 + uv.y * st2;
    }
    // member variables
    uint32_t numTris;                       // number of triangles
    std::unique_ptr<Vec3f []> P;            // triangles vertex position
    std::unique_ptr<uint32_t []> trisIndex; // vertex index array
    std::unique_ptr<Vec3f []> N;            // triangles vertex normals
    std::unique_ptr<Vec2f []> sts;          // triangles texture coordinates
    bool smoothShading = true;              // smooth shading by default
};

TriangleMesh* loadPolyMeshFromFile(const char *file, const Matrix44f &o2w)
{
    std::ifstream ifs;
    try {
        ifs.open(file);
        if (ifs.fail()) throw;
        std::stringstream ss;
        ss << ifs.rdbuf();
        uint32_t numFaces;
        ss >> numFaces;
        std::unique_ptr<uint32_t []> faceIndex(new uint32_t[numFaces]);
        uint32_t vertsIndexArraySize = 0;
        // reading face index array
        for (uint32_t i = 0; i < numFaces; ++i) {
            ss >> faceIndex[i];
            vertsIndexArraySize += faceIndex[i];
        }
        std::unique_ptr<uint32_t []> vertsIndex(new uint32_t[vertsIndexArraySize]);
        uint32_t vertsArraySize = 0;
        // reading vertex index array
        for (uint32_t i = 0; i < vertsIndexArraySize; ++i) {
            ss >> vertsIndex[i];
            if (vertsIndex[i] > vertsArraySize) vertsArraySize = vertsIndex[i];
        }
        vertsArraySize += 1;
        // reading vertices
        std::unique_ptr<Vec3f []> verts(new Vec3f[vertsArraySize]);
        for (uint32_t i = 0; i < vertsArraySize; ++i) {
            ss >> verts[i].x >> verts[i].y >> verts[i].z;
        }
        // reading normals
        std::unique_ptr<Vec3f []> normals(new Vec3f[vertsIndexArraySize]);
        for (uint32_t i = 0; i < vertsIndexArraySize; ++i) {
            ss >> normals[i].x >> normals[i].y >> normals[i].z;
        }
        // reading st coordinates
        std::unique_ptr<Vec2f []> st(new Vec2f[vertsIndexArraySize]);
        for (uint32_t i = 0; i < vertsIndexArraySize; ++i) {
            ss >> st[i].x >> st[i].y;
        }
        
        return new TriangleMesh(o2w, numFaces, faceIndex, vertsIndex, verts, normals, st);
    }
    catch (...) {
        ifs.close();
    }
    ifs.close();
    
    return nullptr;
}


class Light
{
    public:
    Light(const Matrix44f &l2w , const Vec3f &c = 1, const float &i = 1) : lightToWorld(l2w),color(c),intensity(i) {}

    virtual ~Light() {}
    virtual void illuminate(const Vec3f& P, Vec3f&, Vec3f&, float& ) const = 0;


    Matrix44f lightToWorld;
    Vec3f color;
    float intensity;
};

class DistantLight : public Light{

Vec3f dir;
public:

    DistantLight(const Matrix44f& l2w, const Vec3f& c = 1, const float &i = 1) : Light(l2w, c , i)
    {
        l2w.multDirMatrix(Vec3f(0,0,-1), dir);
        dir.normalize();
    }

    void illuminate(const Vec3f& P , Vec3f& lightDir, Vec3f& lightIntensity, float &distance) const
    {
        lightDir = dir;
        lightIntensity = color * intensity;
        distance = kInfinity;
    }
};

class PointLight : public Light 
{
    Vec3f pos;
    public:

    PointLight(const Matrix44f& l2w , const Vec3f& c = 1, const float &i = 1) : Light(l2w, c , i)
    { l2w.multVecMatrix(Vec3f(0), pos);}

    void illuminate(const Vec3f& P , Vec3f &lightDir , Vec3f& lightIntensity, float& distance) const
    {
        lightDir = ( P - pos);
        float r2 = lightDir.norm();
        distance = sqrt(r2);
        lightDir.x /= distance , lightDir.y /= distance , lightDir.z /= distance;

        //avoid division by 0
        if(r2 != 0)
        {
          lightIntensity = (color * intensity) * (1 / (4 * M_PI * r2));
        }
        else
        {
            lightIntensity = Vec3f(0);
        }
    }
};

enum RayType { kPrimaryRay , kShadowRay};

struct IsectInfo
{
    const Object* hitObject = nullptr;
    float tNear = kInfinity;
    Vec2f uv;
    uint32_t index = 0;
};

bool trace(

    const Vec3f& orig , const Vec3f& dir,
    const std::vector<std::unique_ptr<Object>> &objects,
    IsectInfo& isect,
    RayType rayType = kPrimaryRay
)
{
    isect.hitObject = nullptr;
    for(uint32_t k = 0; k < objects.size() ; ++k)
    {
        float tNear = kInfinity;
        uint32_t index = 0;
        Vec2f uv;

        if(objects[k]->intersect(orig,dir,tNear,index,uv) && tNear < isect.tNear)
        {
            isect.hitObject = objects[k].get();
            isect.tNear = tNear;
            isect.index = index;
            isect.uv = uv;
        }
    }


    return (isect.hitObject != nullptr);
}


void createCoordinateSystem(const Vec3f& N,Vec3f& Nt,  Vec3f& Nb)
{
    if(std::fabs(N.x) > std::fabs(N.y))
    {
        Nt = Vec3f(N.z , 0 , -N.x) * (  1 / sqrtf(N.x * N.x + N.z * N.z) );
    }
    else
    {
        Nt = Vec3f(0 , -N.z , N.y) * ( 1 / sqrtf(N.y * N.y + N.z * N.z) );
    }

    Nb = N.crossProduct(Nt);
}

Vec3f uniformSampleHemisphere(const float& r1 , const float& r2)
{
    float sinTheta = sqrtf(1 - r1 * r1);
    float phi = 2 * M_PI * r2;
    float x = sinTheta * cosf(phi);
    float z = sinTheta * sinf(phi);
 
    return Vec3f(x , r1 , z);
}

std::default_random_engine generator;
std::uniform_real_distribution<float> distribution(0,1);


Vec3f castRay(
    const Vec3f& orig,
    const Vec3f& dir,
    const std::vector<std::unique_ptr<Object>>& objects,
    const std::vector<std::unique_ptr<Light>>& lights,
    const Options& options,
    const uint32_t &depth = 0
    )
{
    if(depth > options.maxDepth) return 0;
    Vec3f hitColor = 0;
    IsectInfo isect;

    if(trace(orig, dir, objects, isect))
    {
        Vec3f hitPoint = orig + dir * isect.tNear;
        Vec3f hitNormal;
        Vec2f hitTexCoordinates;
        isect.hitObject->getSurfaceProperties(hitPoint,dir, isect.index, isect.uv, hitNormal, hitTexCoordinates);

        switch(isect.hitObject->type)
        {
            
            case kDiffuse:
            {
                Vec3f directLighting = 0;
                for(uint32_t i = 0 ; i < lights.size(); ++i)
                {
                    Vec3f lightDir, lightIntensity;
                    IsectInfo isectShad;
                    lights[i]->illuminate(hitPoint,lightDir, lightIntensity,isectShad.tNear);
                    bool vis  = !trace(hitPoint + hitNormal * options.bias,-lightDir,objects,isectShad,kShadowRay);

                    directLighting = vis * lightIntensity * std::max(0.f , hitNormal.dotProduct(-lightDir));
                }
                Vec3f indirectLighting = 0;
#ifdef GI
                //Monte Carlo Integration for indirect lighting
                uint32_t N = 8;
                Vec3f Nt , Nb;
                createCoordinateSystem(hitNormal,Nt,Nb);
                float pdf = 1 / (2 * M_PI);
                for(uint32_t n = 0; n < N ; ++n)
                {
                    float r1 = distribution(generator);
                    float r2 = distribution(generator);
                    Vec3f sample = uniformSampleHemisphere(r1,r2);

                    Vec3f sampleWorld(
                        sample.x * Nb.x + sample.y * hitNormal.x + sample.z * Nt.x,
                        sample.x * Nb.y + sample.y * hitNormal.y + sample.z * Nt.y,
                        sample.x * Nb.z + sample.y * hitNormal.z + sample.z * Nt.z);

                    indirectLighting = indirectLighting + ( r1 * castRay(hitPoint + sampleWorld * options.bias, sampleWorld,objects,lights,options,depth+1) * ( 1/ pdf) );
                }

                indirectLighting /= (float)N;
#endif
                hitColor = ( (directLighting * (1 / M_PI)) + 2 * indirectLighting) * isect.hitObject->albedo;
                break;
            }
            default:
                break;
        }
    }
    else{
        hitColor = 1;
    }




    return hitColor;


}



//renders into the framebuffer by casting rays into the scene
void render(const Options& op , 
            const std::vector<std::unique_ptr<Object>> &objects,
            const std::vector<std::unique_ptr<Light>> &lights)
{
    Vec3f* framebuffer = new Vec3f[op.width * op.height];
    Vec3f* pix = framebuffer;

    float aspectRatio = op.width / (float)op.height;
    float scale = tan(degToRad(op.fov * 0.5));



    Vec3f orig;
    op.cameraToWorld.multVecMatrix(Vec3f(0),orig);


    auto s_time = std::chrono::high_resolution_clock::now();

    //for each pixel in my raster space
    for(uint32_t j = 0; j < op.height; ++j)
    {
        for(uint32_t i = 0; i < op.width ; ++i)
        {
            //create a camera ray
            float x =  (2.0 * (i + 0.5) / (float)op.width - 1.0 ) * aspectRatio * scale;
            float y = ( 1.0 - 2.0 * (j + 0.5) / (float)op.height) * scale;
            Vec3f dir;
            op.cameraToWorld.multDirMatrix(Vec3f(x,y,-1.),dir);
            dir.normalize();
            *(pix++) = castRay(orig, dir, objects, lights, op);
        }

        fprintf(stderr,"\r%3d%c",uint32_t(j / (float)op.height * 100), '%');
    }

    
    auto e_time = std::chrono::high_resolution_clock::now();

    auto p_time = std::chrono::duration<float,std::milli>(e_time - s_time).count();

    std::cout << "\nRender Time: " << p_time / 1000 << " (sec)" << std::endl;


    //write image
    std::ofstream ofs;
    ofs.open("GI_indirectLighting.ppm",std::ios::out | std::ios::binary);
    ofs << "P6\n" << op.width << " " << op.height << "\n255\n";
    
    //for each pixel in framebuffer
    for(uint32_t p = 0; p < op.width * op.height; p++)
    {
        unsigned char r = (unsigned char) ( clamp(0.f,1.f,powf(framebuffer[p].x,oneOverGamma)) * 255);
        unsigned char g = (unsigned char) ( clamp(0.f,1.f,powf(framebuffer[p].y,oneOverGamma)) * 255);
        unsigned char b = (unsigned char) ( clamp(0.f,1.f,powf(framebuffer[p].z,oneOverGamma)) * 255);
        
        ofs << r << g << b;
    }

    ofs.close();

    delete [] framebuffer;
}

Matrix44f kIdentity;

int main()
{
    std::vector<std::unique_ptr<Object>> objects;
    std::vector<std::unique_ptr<Light>> lights;
    Options options;

    options.fov = 39.89;
    options.width = 512;
    options.height = 512;
    options.cameraToWorld = Matrix44f(0.965926, 0, -0.258819, 0, 0.0066019, 0.999675, 0.0246386, 0, 0.258735, -0.0255078, 0.965612, 0, 0.764985, 0.791882, 5.868275, 1); 
    
    TriangleMesh* plane = loadPolyMeshFromFile("planegi.geo",kIdentity);
    if(plane != nullptr)
    {
        plane->albedo = Vec3f(0.225,0.144,0.144);
        objects.push_back(std::unique_ptr<Object>(plane));
    }

    TriangleMesh* cube = loadPolyMeshFromFile("cubegi.geo",kIdentity);
    if(cube != nullptr)
    {
        cube->albedo = Vec3f(0.188559,0.287,0.200726);
        objects.push_back(std::unique_ptr<Object>(cube));
    }

    Matrix44f xformSphere;
    xformSphere[3][1] = 1;
    Sphere* sp = new Sphere(xformSphere,1);
    objects.push_back(std::unique_ptr<Object>(sp));


    Matrix44f l2w(0.916445, -0.218118, 0.335488, 0, 0.204618, -0.465058, -0.861309, 0, 0.343889, 0.857989, -0.381569, 0, 0, 0, 0, 1);
    lights.push_back(std::unique_ptr<Light>(new DistantLight(l2w,1,1)));


 

    render(options,objects,lights);

    return 0;
}
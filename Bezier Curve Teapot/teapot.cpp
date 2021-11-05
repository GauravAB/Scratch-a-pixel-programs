#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <cassert>
#include <cstddef>
#include <vector>
#include <cmath>
#include <random>
#include <sstream>
#include <cstdint>
#include <utility>
#include <algorithm>
#include <chrono>

#include "geometry.h"
#include "teapotdata.h"

static const float kInfinity = std::numeric_limits<float>::max();
static const float kEpsilon = 1e-8;
static const Vec3f kDefaultBackgroundColor = Vec3f(0.235294,0.67451,0.843137);

const Matrix44f kIdentity = Matrix44f();

inline float clamp(const float& lo , const float& hi, const float& v)
{
    return std::max(lo,std::min(hi,v));
}

inline 
float deg2rad(const float &deg)
{
    return deg * M_PI / 180;
}

inline Vec3f mix(const Vec3f &a, const Vec3f &b ,const float &mixValue)
{
    return a * (1 - mixValue) + b * mixValue;
}

struct Options
{
    uint32_t width = 640;
    uint32_t height = 480;
    float fov = 90;
    Vec3f backgroundColor = kDefaultBackgroundColor;
    Matrix44f cameraToWorld;
    float bias = 0.0001;
    uint32_t maxDepth = 2;
};

enum MaterialType 
{
    kDiffuse, kNothing
};

//something which our rays can probe
class Object
{   
    public:
    Object(const Matrix44f &o2w) : objectToWorld(o2w), worldToObject(o2w.inverse()) {}
    virtual ~Object(){}
    virtual bool intersect(const Vec3f&, const Vec3f&, float&, uint32_t & , Vec2f&) const = 0;
    virtual void getSurfaceProperties(const Vec3f& , const Vec3f& , const uint32_t & ,const Vec2f& , Vec3f&, Vec2f&)const = 0;
    virtual void displayInfo() const = 0;
    Matrix44f objectToWorld , worldToObject;
    MaterialType type = kDiffuse;
    
    
    Vec3f albedo = 0.18;
    float Kd = 0.8;
    float Ks = 0.2;
    float n = 10;
    Vec3f BBox[2] = {kInfinity, -kInfinity};
};

bool rayTriangleIntersect(
    const Vec3f& orig ,const Vec3f &dir, 
    const Vec3f& v0, const Vec3f& v1, const Vec3f& v2,
    float &t ,float& u, float &v)
    {
        Vec3f v0v1 = v1 - v0;
        Vec3f v0v2 = v2 - v0;
        Vec3f pvec = dir.crossProduct(v0v2);
        float det = v0v1.dotProduct(pvec);

        //ray and triangle paralled if det is close to 0
        if(fabs(det) < kEpsilon) return false;

        float invDet = 1 / det;

        Vec3f tvec = orig - v0;
        u = tvec.dotProduct(pvec) * invDet;
        if(u < 0 || u > 1) return false;

        Vec3f qvec = tvec.crossProduct(v0v1);
        v = dir.dotProduct(qvec) * invDet;
        if(v < 0 || u + v > 1) return false;

        t = v0v2.dotProduct(qvec)* invDet;

        return (t > 0) ? true : false;

    }

class TriangleMesh : public Object{

public:

    //build a triangle mesh from a face index array and vertex index array
    TriangleMesh(
        const Matrix44f &o2w,
        const uint32_t nfaces,
        const std::unique_ptr<uint32_t[]>& faceIndex,
        const std::unique_ptr<uint32_t[]>& vertsIndex,
        const std::unique_ptr<Vec3f[]>& verts,
        std::unique_ptr<Vec3f[]>& normals,
        std::unique_ptr<Vec2f[]>& st,
        bool singleVertAtrr = true
    ) : Object(o2w),numTris(0),isSingleVertAttr(singleVertAtrr)
    {
        uint32_t k = 0, maxVertIndex = 0;
        //find out how many triangles we need to create for this mesh

        for(uint32_t i = 0; i < nfaces ; ++i)
        {
            numTris += faceIndex[i] - 2;
            for(uint32_t j = 0; j < faceIndex[i]; ++j)
            {
                if(vertsIndex[k + j] > maxVertIndex)
                {
                    maxVertIndex = vertsIndex[ k + j];
                }
            }
            k += faceIndex[i];
        }

        maxVertIndex += 1;

        //allocate memory to store the position of the mesh vertices
        P = std::unique_ptr<Vec3f[]>(new Vec3f[maxVertIndex]);

        for(uint32_t i = 0 ; i < maxVertIndex; ++i)
        {
            objectToWorld.multVecMatrix(verts[i], P[i]);
        }
        
        //allocate memory to store triangle indices
        trisIndex = std::unique_ptr<uint32_t []>(new uint32_t [numTris * 3]);
        Matrix44f transformNormals = worldToObject.transpose();

        if(isSingleVertAttr)
        {
            N = std::unique_ptr<Vec3f[]>(new Vec3f[maxVertIndex]);
            texCoordinates = std::unique_ptr<Vec2f[]>(new Vec2f[maxVertIndex]);
            for(uint32_t i = 0; i < maxVertIndex; ++i)
            {
                texCoordinates[i] = st[i];
                transformNormals.multDirMatrix(normals[i],N[i]);
            }
        }
        else
        {
            N = std::unique_ptr<Vec3f[]>(new Vec3f[numTris * 3]);
            texCoordinates = std::unique_ptr<Vec2f[]>(new Vec2f[numTris * 3]);
            for(uint32_t i = 0, k = 0, l = 0; i < nfaces; ++i)
            {
                for(uint32_t j = 0; j < faceIndex[i] -2 ; ++j)
                {
                    transformNormals.multDirMatrix(normals[k], N[l]);
                    transformNormals.multDirMatrix(normals[k + j + 1], N[l + 1]);
                    transformNormals.multDirMatrix(normals[k + j + 2],N[l + 2]);
                    N[l].normalize();
                    N[l + 1].normalize();
                    N[l + 2].normalize();
                    texCoordinates[l] = st[k];
                    texCoordinates[l + 1] = st[k + j + 1];
                    texCoordinates[l + 2] = st[k + j + 2];
                }

                k += faceIndex[i];
            }
        }

        //generate the triangle index array and set normals and st coordinates
        for(uint32_t i = 0 , k = 0, l = 0; i < nfaces; ++i)
        {
            for(uint32_t j = 0; j < faceIndex[i]- 2; ++j)
            {
                trisIndex[l] = vertsIndex[k];
                trisIndex[l + 1] = vertsIndex[k + j + 1];
                trisIndex[l + 2] = vertsIndex[k + j + 2];

                l += 3;
            }

            k += faceIndex[i];
        }
    }

    //Test if the ray intersects this triangle mesh
    bool intersect(const Vec3f& orig, const Vec3f& dir, float& tNear, uint32_t &triIndex , Vec2f& uv) const
    {
        uint32_t j = 0;
        bool isect = false;
        for(uint32_t i = 0; i < numTris; ++i)
        {
            const Vec3f &v0  = P[trisIndex[j]];
            const Vec3f &v1  = P[trisIndex[j + 1]];
            const Vec3f &v2  = P[trisIndex[j + 2]];
            float t = kInfinity, u ,  v;

            if(rayTriangleIntersect(orig , dir , v0, v1, v2, t , u, v) && t < tNear)
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
        const Vec3f &hitPoint,
        const Vec3f &viewDirection,
        const uint32_t &triIndex,
        const Vec2f &uv,
        Vec3f &hitNormal,
        Vec2f &hitTextureCoordinates) const
    {
        uint32_t vai[3]; //vertex attr index
        if(isSingleVertAttr)
        {
            vai[0] = trisIndex[triIndex * 3];
            vai[1] = trisIndex[triIndex * 3 + 1];
            vai[2] = trisIndex[triIndex * 3 + 2];           
        }
        else
        {
            vai[0] = triIndex * 3;
            vai[1] = triIndex * 3 + 1;
            vai[2] = triIndex * 3 + 2;

        }

        if(smoothShading)
        {
            //vertex normal
            const Vec3f &n0 = N[vai[0]];
            const Vec3f &n1 = N[vai[1]];
            const Vec3f &n2 = N[vai[2]];

            hitNormal = (1 - uv.x - uv.y) * n0 + uv.x * n1 + uv.y * n2;
        }
        else
        {
            //face normal
            const Vec3f& v0 = P[trisIndex[triIndex * 3]];
            const Vec3f& v1 = P[trisIndex[triIndex * 3 + 1]];
            const Vec3f& v2 = P[trisIndex[triIndex * 3 + 2]];
            hitNormal = (v1 - v0).crossProduct(v2 - v0);
        }

        //doesn't need to be normalized as the N's are normalized but just for safety
        hitNormal.normalize();

        //texture coordinates
        const Vec2f &st0 = texCoordinates[vai[0]];
        const Vec2f &st1 = texCoordinates[vai[1]];
        const Vec2f &st2 = texCoordinates[vai[2]];

        hitTextureCoordinates = ( 1 - uv.x - uv.y) * st0 + uv.x * st1 + uv.y * st2;
    }

    void displayInfo() const{
        std::cerr << "Number of triangles in this mesh: " << numTris << std::endl;
        std::cerr << BBox[0] << ", " << BBox[1] << std::endl;
    }

    //member variables
    uint32_t numTris ;
    std::unique_ptr<Vec3f[]> P;
    std::unique_ptr<uint32_t[]> trisIndex;
    std::unique_ptr<Vec3f[]>N;
    std::unique_ptr<Vec2f[]> texCoordinates;
    bool smoothShading = true;
    bool isSingleVertAttr = true;
};

class Light
{
    public:

    Light(const Matrix44f &l2w, const Vec3f &c = 1, const float& i = 1) : lightToWorld(l2w), color(c), intensity(i){}
    virtual ~Light() {}
    virtual void illuminate(const Vec3f& P, Vec3f& , Vec3f&, float& ) const = 0;
    Vec3f color;
    float intensity;
    Matrix44f lightToWorld;  
};

struct IsectInfo
{
    const Object* hitObject = nullptr;
    float tNear = kInfinity;
    Vec2f uv;
    uint32_t index = 0;
};

bool trace(
    const Vec3f& orig, const Vec3f& dir,
    const std::vector<std::unique_ptr<Object>>& objects,
    IsectInfo& isect
    )
    {
        isect.hitObject = nullptr;
        for(uint32_t k = 0; k < objects.size(); ++k)
        {
            float tNearTriangle = kInfinity;
            uint32_t indexTriangle;
            Vec2f uvTriangle;
            if(objects[k]->intersect(orig, dir, tNearTriangle,indexTriangle,uvTriangle) && tNearTriangle < isect.tNear)
            {
                isect.hitObject = objects[k].get();
                isect.tNear = tNearTriangle;
                isect.index = indexTriangle;
                isect.uv = uvTriangle;
            }
        }

        return (isect.hitObject != nullptr);
    }

Vec3f castRay(
    const Vec3f &orig, const Vec3f& dir,
    const std::vector<std::unique_ptr<Object>>& objects,
    const std::vector<std::unique_ptr<Light>>& lights,
    const Options &options,
    const uint32_t& depth = 0)
{
    if(depth > options.maxDepth) return 0;
    Vec3f hitColor = 0;
    IsectInfo isect;

    if(trace(orig,dir,objects,isect))
    {
        Vec3f hitPoint = orig + dir * isect.tNear;
        Vec3f hitNormal;
        Vec2f hitTexCoordinates;
        isect.hitObject->getSurfaceProperties(hitPoint,dir,isect.index,isect.uv, hitNormal,hitTexCoordinates);

        hitColor = std::max(0.f, -hitNormal.dotProduct(dir));
    }
    else{
        hitColor = 0.2;
    }

    return hitColor;
}


void render(
    const Options& options,
    const std::vector<std::unique_ptr<Object>> &objects,
    const std::vector<std::unique_ptr<Light>> &lights)
{
    Vec3f *framebuffer = new Vec3f[options.width * options.height];
    Vec3f* pix = framebuffer;
    float scale = tan(deg2rad(options.fov * 0.5));
    float imageAspectRatio = options.width / (float)options.height;
    Vec3f orig;
    options.cameraToWorld.multVecMatrix(Vec3f(0),orig);

    auto timeStart = std::chrono::high_resolution_clock::now();

    for(uint32_t j = 0 ; j < options.height ; j++)
    {
        for(uint32_t i = 0; i < options.width; i++)
        {
            //generate primary ray direction
            float x =  (2.0 * (i + 0.5)/(float)options.width - 1) * imageAspectRatio * scale;
            float y = (1 - 2.0*(j + 0.5) / (float)options.height) * scale;
            Vec3f dir;
            options.cameraToWorld.multDirMatrix(Vec3f(x , y , -1) , dir);
            dir.normalize();
            *(pix++) = castRay(orig, dir, objects, lights, options);
        }

        fprintf(stderr, "\r%3d%c",uint32_t(j /(float)options.height * 100), '%');
    }

    auto timeEnd = std::chrono::high_resolution_clock::now();
    auto passedTime = std::chrono::duration<float,std::milli>(timeEnd - timeStart).count();
    fprintf(stderr, "\rDone: %.2f (sec)\n",passedTime/1000);

    //save framebuffer to file
    float gamma = 1;
    std::ofstream ofs;
    ofs.open("out.ppm");
    ofs << "P6\n" << options.width << " " << options.height << "\n255\n";

    for(uint32_t k = 0 ; k < options.width * options.height; k++)
    {
        char r = (char)(255 * clamp(0,1,powf(framebuffer[k].x,1/gamma)));
        char g = (char)(255 * clamp(0,1,powf(framebuffer[k].y,1/gamma)));
        char b = (char)(255 * clamp(0,1,powf(framebuffer[k].z,1/gamma)));
        
        ofs << r << g << b;
    }

    ofs.close();
    delete [] framebuffer;
}

Vec3f evalBezierCurve(const Vec3f *P, const float &t)
{
    float b0 = (1-t) * (1-t) * (1-t);
    float b1 = 3 * t * (1-t) * (1-t);
    float b2 = 3 * t * t * ( 1-t);
    float b3 = t*t*t;

    return P[0] * b0 + P[1] * b1 + P[2] * b2 + P[3] * b3;
}

Vec3f evalBezierPatch(const Vec3f* controlPoints, const float& u, const float& v)
{
    Vec3f uCurve[4];
    for(int i = 0; i < 4; ++i)
    {
        uCurve[i] = evalBezierCurve(controlPoints + 4 * i , u);
    }

    return evalBezierCurve(uCurve,v);
}

Vec3f derivBezier(const Vec3f *P, const float &t)
{
    return -3 * (1-t) * (1-t) * P[0] +
            (3 * (1-t)*(1-t)-6*t*(1-t)) * P[1] +
            (6*t*(1-t)-3*t*t) *P[2] +
            3 *t *t * P[3];
}

//compute the derivative of a point on Bezier patch along the u parametric direction
Vec3f dUBezier(const Vec3f* controlPoints, const float& u , const float& v)
{
    Vec3f P[4];
    Vec3f vCurve[4];
    for(int i = 0; i < 4; ++i)
    {
        P[0] = controlPoints[i];
        P[1] = controlPoints[4 + i];
        P[2] = controlPoints[8 + i];
        P[3] = controlPoints[12 + i];
        
        vCurve[i] = evalBezierCurve(P, v);
    }

    return derivBezier(vCurve, u);
}

Vec3f dVBezier(const Vec3f* controlPoints, const float& u , const float& v)
{
    Vec3f uCurve[4];
    for(int i = 0; i < 4 ; ++i)
    {
        uCurve[i] = evalBezierCurve(controlPoints + 4 * i, u);
    }

    return derivBezier(uCurve, v);
}

//creat utah teapot
void createPolyTeapot(const Matrix44f& o2w, std::vector<std::unique_ptr<Object>>& objects)
{
    uint32_t divs = 8;
    std::unique_ptr<Vec3f[]>P(new Vec3f[(divs + 1) * (divs + 1)]);
    std::unique_ptr<uint32_t[]>nvertices(new uint32_t[divs * divs]);
    std::unique_ptr<uint32_t[]>vertices(new uint32_t[divs * divs * 4]);
    std::unique_ptr<Vec3f[]> N(new Vec3f[(divs + 1) * (divs + 1)]);
    std::unique_ptr<Vec2f[]> st(new Vec2f[(divs + 1) * (divs + 1)]);

    //Face connectivity
    //All patches are subdivided the same way so they share same topology and uvs
    for(uint16_t j =0 , k =0; j < divs ; ++j)
    {
        for(uint16_t i =0; i < divs; ++i, ++k)
        {
            nvertices[k] = 4;
            vertices[k * 4] = (divs + 1) * j + i;
            vertices[k * 4 + 1] = (divs + 1) * j + i + 1;
            vertices[k * 4 + 2] = (divs + 1) * (j + 1) + i + 1;

        }
    }

    Vec3f controlPoints[16];
    for(int np = 0; np < kTeapotNumPatches; ++np)
    {
        //set control points for the current patch
        for(uint32_t i = 0; i < 16 ; ++i)
        {
            controlPoints[i][0] = teapotVertices[teapotPatches[np][i] - 1][0],
            controlPoints[i][1] = teapotVertices[teapotPatches[np][i] - 1][1],
            controlPoints[i][2] = teapotVertices[teapotPatches[np][i] - 1][2];
        }

        //generate grid
        for(uint16_t j = 0, k =0 ; j <= divs; ++j)
        {
            float v = j / (float)divs;
            for(uint16_t i = 0; i <= divs; ++i , ++k)
            {
                float u = i / (float)divs;
                P[k] = evalBezierPatch(controlPoints, u , v);
                Vec3f dU = dUBezier(controlPoints, u , v);
                Vec3f dV = dVBezier(controlPoints,u ,v);
                N[k] = dU.crossProduct(dV).normalize();
                st[k].x = u;
                st[k].y = v;
            }
        }

        objects.push_back(std::unique_ptr<TriangleMesh>(new TriangleMesh(o2w,divs * divs , nvertices,vertices,P,N,st)));
    }
}
        
int main(int argc , char** argv )
{
    //load geometry
    std::vector<std::unique_ptr<Object>> objects;

    createPolyTeapot(Matrix44f(1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1),objects);

    //lights
    std::vector<std::unique_ptr<Light>> lights;
    Options options;

    options.fov = 39.89;
    options.width = 512;
    options.height = 512;
    options.maxDepth = 1;

    //render the teapot
    options.cameraToWorld = Matrix44f(0.897258, 0, -0.441506, 0, -0.288129, 0.757698, -0.585556, 0, 0.334528, 0.652606, 0.679851, 0, 5.439442, 11.080794, 10.381341, 1); 
 
    render(options,objects,lights);

    return 0;
}
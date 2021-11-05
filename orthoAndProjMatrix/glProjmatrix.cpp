#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <algorithm>
#include "geometry.h"
#include "vertexdata.h"

#ifndef M_PI
#define M_PI 3.14159265
#endif

void gluPerspective(
	const float& angleOfView,
	const float& imageAspectRatio,
	const float& n, const float& f,
	float& b, float& t, float& l, float& r
)
{
	float scale = tan(angleOfView * 0.5 * M_PI / 180) * n;
	r = imageAspectRatio * scale;
	t = scale;
	l = -r;
	b = -t;
}

void glFrustum(
	const float& b, const float& t, const float& l, const float& r,
	const float& n, const float& f, Matrix44f& M
)
{
	M[0][0] = 2 * n / (r - l);
	M[0][1] = 0;
	M[0][2] = 0;
	M[0][3] = 0;

	M[1][0] = 0;
	M[1][1] = 2 * n / (t - b);
	M[1][2] = 0;
	M[1][3] = 0;

	M[2][0] = (r + l) / (r - l);
	M[2][1] = (t + b) / (t - b);
	M[2][2] = -(f + n) / (f - n);
	M[2][3] = -1;


	M[3][0] = 0;
	M[3][1] = 0;
	M[3][2] = -2 * f * n / (f - n);
	M[3][3] = 0;	
}


void multPointMatrix(const Vec3f& in, Vec3f& out, const Matrix44f& M)
{
	//since vec3f has w = 1 assumed
	out.x = in.x * M[0][0] + in.y * M[1][0] + in.z * M[2][0] + M[3][0];
	out.y = in.x * M[0][1] + in.y * M[1][1] + in.z * M[2][1] + M[3][1];
	out.z = in.x * M[0][2] + in.y * M[1][2] + in.z * M[2][2] + M[3][2];
	
	float w = in.x * M[0][3] + in.y * M[1][3] + in.z * M[2][3] + M[3][3];

	if (w != 1)
	{
		out.x /= w;
		out.y /= w;
		out.z /= w;

	}
}

int main(int argc, char** argv)
{
	uint32_t imageWidth = 512;
	uint32_t imageHeight = 512;

	Matrix44f Mproj;
	Matrix44f worldToCamera;
	worldToCamera[3][1] = -10;
	worldToCamera[3][2] = -20;
	float angleOfView = 90;
	float near = 0.1;
	float far = 100;
	float imageAspectRatio = imageWidth / (float)imageHeight;
	float b, t, l, r;

	gluPerspective(angleOfView, imageAspectRatio, near, far, b, t, l, r);
	glFrustum(b, t, l, r, near, far, Mproj);
	unsigned char* buffer = new unsigned char[imageWidth * imageHeight];
	memset(buffer, 0x0, imageWidth * imageHeight);
	
	for (uint32_t i = 0; i < numVertices; ++i)
	{
		Vec3f vertCamera, projectedVert;

		multPointMatrix(vertices[i], vertCamera, worldToCamera);
		multPointMatrix(vertCamera, projectedVert, Mproj);

		if (projectedVert.x < -imageAspectRatio || projectedVert.x > imageAspectRatio || projectedVert.y < -1 || projectedVert.y > 1) continue;
		
		uint32_t x = std::min(imageWidth - 1, (uint32_t)((projectedVert.x + 1.0) * 0.5 * imageWidth));
		uint32_t y = std::min(imageHeight - 1, (uint32_t)((1.0 - projectedVert.y) * 0.5 * imageHeight));
		buffer[y * imageWidth + x] = 255;

	}

	//export to an image
	std::ofstream ofs;
	ofs.open("out1.ppm");
	ofs << "P5\n" << imageWidth << " " << imageHeight << "\n255\n";
	ofs.write((char*)buffer, imageWidth * imageHeight);
	ofs.close();

	delete[] buffer;



	return 0;
}







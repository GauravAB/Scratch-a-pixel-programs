#include <cstdio>
#include <cstdlib>
#include "geometry.h"
#include "vertexdata.h"

void setProjectionMatrix(const float& angleOfView, const float& near, const float& far, Matrix44f& M)
{
	//set canvas coordinates and fill the projection matrix
	float scale = 1 / 
}
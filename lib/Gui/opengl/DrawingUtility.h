#ifndef DRAWINGUTILITY_H_
#define DRAWINGUTILITY_H_

#ifdef USE_OPENSIM

#include "SimTKcommon.h"

#ifdef WIN32
#include <windows.h>
#include <GL/glew.h>
//#include <GL/gl.h>
#include <GL/glu.h>
#endif

#define GL_GLEXT_PROTOTYPES

#ifdef UNIX
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <GL/glut.h>
#endif
#ifdef APPLE
#include <gl.h>
#include <glu.h>
#include <glext.h>
#include <Glut/glut.h>
#endif


#include <cstdlib>
#include <string>
#include <algorithm>
#include <set>
#include <vector>
#include <utility>
#include <limits>
#include <cstdio>
#include <cerrno>
#include <cstring>

using namespace SimTK;
using namespace std;

static void computeBoundingSphereForVertices(const vector<float>& vertices,
		float& radius, Vec3& center)
{
	fVec3 lower(vertices[0], vertices[1], vertices[2]);
	fVec3 upper = lower;
	for (int i = 3; i < (int) vertices.size(); i += 3)
	{
		for (int j = 0; j < 3; j++)
		{
			lower[j] = min(lower[j], vertices[i + j]);
			upper[j] = max(upper[j], vertices[i + j]);
		}
	}
	center = (lower + upper) / 2;
	float radians2 = 0;
	for (int i = 0; i < (int) vertices.size(); i += 3)
	{
		float x = center[0] - vertices[i];
		float y = center[1] - vertices[i + 1];
		float z = center[2] - vertices[i + 2];
		float norm2 = x * x + y * y + z * z;
		if (norm2 > radians2)
			radians2 = norm2;
	}
	radius = sqrt(radians2);
}



//static void drawSkyVertex(fVec3 position, float texture)
//{
//	glTexCoord1f(texture);
//	glVertex3fv(&position[0]);
//}

#endif
#endif

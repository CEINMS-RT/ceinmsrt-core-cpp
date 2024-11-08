/*
 * Mesh.h
 *
 *  Created on: Jun 26, 2015
 *      Author: gdurandau
 */

#ifndef MESH_H_
#define MESH_H_

#ifdef USE_OPENSIM



#include "SimTKcommon.h"
/*/ /!\ Warning do not put
//#include "SimTKcommon.h" or other opensim simtk library

after
#include <QtOpenGL/QtOpenGL>
#include <QtOpenGL/QGLWidget>

#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <GL/glut.h>

will not compile because of redinition but work fine on Linux (go figure !)

*/
#include <DrawingUtility.h>

#define GL_GLEXT_PROTOTYPES
#ifdef WIN32
#include <windows.h>
#include <GL/glew.h>
//#include <GL/gl.h>
#include <GL/glu.h>
#endif



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

using namespace std;

// Windows special function because windows is the special "kid"
#ifdef WIN32

#endif

class Mesh {
public:
	Mesh() {}
	Mesh(vector<float>& vertices, vector<float>& normals, vector<GLushort>& faces);
	virtual ~Mesh();

	void draw(short representation) const;
	void getBoundingSphere(float& radius, fVec3& center);

private:
    int numVertices;
    GLuint vertBuffer, normBuffer;
    vector<GLushort> edges, faces;
    Vec3 center;
    float radius;
};

#endif
#endif /* MESH_H_ */

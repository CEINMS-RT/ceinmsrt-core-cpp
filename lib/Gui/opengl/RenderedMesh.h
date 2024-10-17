/*
 * RenderedMesh.h
 *
 *  Created on: Jun 26, 2015
 *      Author: gdurandau
 */

#ifndef RENDEREDMESH_H_
#define RENDEREDMESH_H_

#ifdef USE_OPENSIM

#include "SimTKcommon.h"
#include <Mesh.h>

#define GL_GLEXT_PROTOTYPES
#ifdef WIN32
#include <windows.h>
#include <GL/gl.h>
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

class RenderedMesh {
public:
	RenderedMesh() {}
	RenderedMesh(const SimTK::Transform& transform, const Vec3& scale,
		const SimTK::Vec4& color, short representation,
			unsigned short meshIndex, unsigned short resolution);
	virtual ~RenderedMesh();
	void draw(bool setColor);

	inline void setMesh(vector<float>& vertices, vector<float>& normals, vector<GLushort>& faces)
	{
		_mesh = Mesh(vertices, normals, faces);
	}

	inline const SimTK::Transform& getTransform() const
	{
		return transform;
	}

	inline void setTransform(const SimTK::Transform& transform)
	{
		this->transform = transform;
	}

	void computeBoundingSphere(float& radius, SimTK::fVec3& center);

private:
	SimTK::Transform transform;
	SimTK::Vec3 scale;
    GLfloat color[4];
    Mesh _mesh;
    short representation;
    unsigned short meshIndex, resolution;
};

#endif
#endif /* RENDEREDMESH_H_ */

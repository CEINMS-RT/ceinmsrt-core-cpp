// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software. Any changes to this code, should be shared back in the open repository: https://github.com/CEINMS-RT. See license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE.
//
// The methodologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots. TechRxiv. DOI: 10.36227/techrxiv.173397962.28177284/v1"
//

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

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

#include "RenderedMesh.h"

#ifdef USE_OPENSIM

RenderedMesh::RenderedMesh(const Transform& transform, const Vec3& scale,
		const Vec4& color, short representation, unsigned short meshIndex,
		unsigned short resolution) :
		transform(transform), scale(scale), representation(representation), meshIndex(
				meshIndex), resolution(resolution)
{
	this->color[0] = color[0];
	this->color[1] = color[1];
	this->color[2] = color[2];
	this->color[3] = color[3];
}

RenderedMesh::~RenderedMesh()
{
	// TODO Auto-generated destructor stub
}

void RenderedMesh::draw(bool setColor = true)
{
	glPushMatrix();
	glTranslated(transform.p()[0], transform.p()[1], transform.p()[2]);
	Vec4 rot = transform.R().convertRotationToAngleAxis();
	glRotated(rot[0] * SimTK_RADIAN_TO_DEGREE, rot[1], rot[2], rot[3]);
	glScaled(scale[0], scale[1], scale[2]);
	if (setColor)
	{
		if (representation == DecorativeGeometry::DrawSurface)
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
		else
			glColor3fv(color);
	}
	_mesh.draw(representation);
	glPopMatrix();
}

void RenderedMesh::computeBoundingSphere(float& radius, fVec3& center)
{
	_mesh.getBoundingSphere(radius, center);
	center += transform.p();
	radius *= max(abs(scale[0]), max(abs(scale[1]), abs(scale[2])));
}

#endif

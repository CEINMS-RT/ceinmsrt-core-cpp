/*
 * RenderedMesh.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: gdurandau
 */

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

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

#include "Mesh.h"

#ifdef USE_OPENSIM
Mesh::Mesh(vector<float>& vertices, vector<float>& normals, vector<GLushort>& faces)
    :   numVertices((int)(vertices.size()/3)), faces(faces) {
        // Build OpenGL buffers.

//	std::cout << "vertices: " << vertices.size() << std::endl;

        GLuint buffers[2];
        glGenBuffers(2, buffers);
        vertBuffer = buffers[0];
        normBuffer = buffers[1];
		glBindBuffer(GL_ARRAY_BUFFER, vertBuffer);
		glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), &vertices[0], GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, normBuffer);
        glBufferData(GL_ARRAY_BUFFER, normals.size()*sizeof(float), &normals[0], GL_STATIC_DRAW);

        // Create the list of edges.

        set<pair<GLushort, GLushort> > edgeSet;
        for (int i = 0; i < (int) faces.size(); i += 3) {
            GLushort v1 = faces[i];
            GLushort v2 = faces[i+1];
            GLushort v3 = faces[i+2];
            edgeSet.insert(make_pair(min(v1, v2), max(v1, v2)));
            edgeSet.insert(make_pair(min(v2, v3), max(v2, v3)));
            edgeSet.insert(make_pair(min(v3, v1), max(v3, v1)));
        }
        for (set<pair<GLushort, GLushort> >::const_iterator iter = edgeSet.begin(); iter != edgeSet.end(); ++iter) {
            edges.push_back(iter->first);
            edges.push_back(iter->second);
        }

        // Compute the center and radius.

        computeBoundingSphereForVertices(vertices, radius, center);
    }

Mesh::~Mesh() {
	// TODO Auto-generated destructor stub
}

void Mesh::draw(short representation) const {
	glBindBuffer(GL_ARRAY_BUFFER, vertBuffer);
    glVertexPointer(3, GL_FLOAT, 0, 0);
	glBindBuffer(GL_ARRAY_BUFFER, normBuffer);
    glNormalPointer(GL_FLOAT, 0, 0);
    if (representation == DecorativeGeometry::DrawSurface)
        glDrawElements(GL_TRIANGLES, (GLsizei)faces.size(), GL_UNSIGNED_SHORT, &faces[0]);
    else if (representation == DecorativeGeometry::DrawPoints)
        glDrawArrays(GL_POINTS, 0, numVertices);
    else if (representation == DecorativeGeometry::DrawWireframe)
        glDrawElements(GL_LINES, (GLsizei)edges.size(), GL_UNSIGNED_SHORT, &edges[0]);
}

void Mesh::getBoundingSphere(float& radius, fVec3& center) {
    radius = this->radius;
    center = this->center;
}
#endif

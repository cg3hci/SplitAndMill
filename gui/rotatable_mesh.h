/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef ROTATABLE_MESH_H
#define ROTATABLE_MESH_H

#include <cg3/viewer/interfaces/manipulable_object.h>
#include <cg3/viewer/drawable_objects/drawable_dcel.h>
#include <cg3/viewer/glcanvas.h>

class RotatableMesh : public cg3::ManipulableObject
{
public:
	RotatableMesh(cg3::viewer::GLCanvas& canvas);
	RotatableMesh(cg3::viewer::GLCanvas& canvas, const cg3::DrawableDcel& mesh);

	void setMesh(const cg3::DrawableDcel &mesh);

	// DrawableObject interface
	void draw() const;
	cg3::Point3d sceneCenter() const;
	double sceneRadius() const;

	// ManipulableObject interface
	void drawHighlighted() const;
	void checkIfGrabsMouse(int x, int y, const qglviewer::Camera * const camera);


private:
	cg3::viewer::GLCanvas& canvas;
	bool init;
	cg3::DrawableDcel mesh;
	cg3::DrawableDcel arrow[3];
	cg3::Color arrowColor[3];
	cg3::Point3d bc[3];
	qglviewer::LocalConstraint constraints[3];
	int grabbedArrow;
};

#endif // ROTATABLE_MESH_H

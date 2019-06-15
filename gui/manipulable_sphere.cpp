/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "manipulable_sphere.h"

#include <cg3/utilities/const.h>
#include <cg3/viewer/opengl_objects/opengl_objects3.h>
#include <cg3/viewer/glcanvas.h>

ManipulableSphere::ManipulableSphere(cg3::viewer::GLCanvas &canvas) :
	center(), radius(0.01), color(cg3::BLUE), colorHighlited(cg3::RED), canvas(canvas)
{
	  //setRotationSensitivity(0);
	  setWheelSensitivity(0);
	  //setMouseBinding(Qt::NoModifier, Qt::LeftButton, QGLViewer::TRANSLATE);
}

void ManipulableSphere::setRadius(double d)
{
	radius = d;
}

void ManipulableSphere::draw() const
{
	cg3::opengl::drawSphere(center, radius, color, 8);
}

void ManipulableSphere::drawHighlighted() const
{
	cg3::opengl::drawSphere(center, radius, colorHighlited, 8);
}

cg3::Point3d ManipulableSphere::sceneCenter() const
{
	return center;
}

double ManipulableSphere::sceneRadius() const
{
	return radius;
}

void ManipulableSphere::checkIfGrabsMouse(int x, int y, const qglviewer::Camera * const camera)
{
	ManipulableObject::checkIfGrabsMouse(x, y, camera);
	if (grabsMouse()){
		canvas.setManipulatedFrame(this);
		canvas.setMouseBinding(Qt::NoModifier, Qt::LeftButton, QGLViewer::FRAME,
						QGLViewer::TRANSLATE);
	}
	else {
		canvas.setManipulatedFrame(camera->frame());
		canvas.setMouseBinding(Qt::NoModifier, Qt::LeftButton, QGLViewer::CAMERA,
						QGLViewer::ROTATE);
	}
}



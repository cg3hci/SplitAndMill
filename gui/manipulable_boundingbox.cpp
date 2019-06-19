/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "manipulable_boundingbox.h"

#include <cg3/utilities/const.h>
#include <cg3/viewer/opengl_objects/opengl_objects3.h>

ManipulableBoundingBox::ManipulableBoundingBox(cg3::viewer::GLCanvas& canvas) :
	canvas(canvas),
	_drawArrow(false),
	df(1),
	_min(canvas),
	_max(canvas),
	millingDir(HFBox::PLUS_Z)
{
	_min.setReferenceFrame(this);
	_max.setReferenceFrame(this);
	setWheelSensitivity(0);
	setRotationSensitivity(0);
	setGrabbingFactor(0.05);
}

void ManipulableBoundingBox::draw() const
{
	drawSpheres();
	cg3::opengl::drawBox3(_min.translation(), _max.translation(), cg3::BLACK, 2);
	drawArrow();
}

cg3::Point3d ManipulableBoundingBox::sceneCenter() const
{
	return (_min.translation() + _max.translation()) * 0.5;
}

double ManipulableBoundingBox::sceneRadius() const
{
	return _min.translation().dist(_max.translation());
}

void ManipulableBoundingBox::drawHighlighted() const
{
	drawSpheres();
	cg3::opengl::drawBox3(_min.translation(), _max.translation(), cg3::RED, 5);
	drawArrow();
}

void ManipulableBoundingBox::set(const cg3::Point3d &min, const cg3::Point3d &max)
{
	cg3::Point3d center = (min + max)*0.5;
	setTranslation(center);
	_min.setTranslation(min-center);
	_max.setTranslation(max-center);
	df = _min.translation().dist(_max.translation()) / 10;
	_min.setRadius(df/6);
	_max.setRadius(df/6);
}

void ManipulableBoundingBox::setMillingDirection(HFBox::MillingDir dir)
{
	millingDir = dir;
}

void ManipulableBoundingBox::drawSpheres() const
{
	// Save the current model view matrix (not needed here in fact)
	glPushMatrix();
	glMultMatrixd(_min.matrix());
	if (_min.grabsMouse())
		_min.drawHighlighted();
	else
		_min.draw();
	glPopMatrix();

	glPushMatrix();
	glMultMatrixd(_max.matrix());
	if (_max.grabsMouse())
		_max.drawHighlighted();
	else
		_max.draw();

	glPopMatrix();
}

void ManipulableBoundingBox::drawArrow() const
{
	if (_drawArrow) {
		cg3::Point3d base = (_min.translation() + _max.translation()) * 0.5;
		cg3::Point3d minBody, maxBody, arrowPoint;
		cg3::Color col;
		switch (millingDir) {
		case HFBox::PLUS_X :
			col = cg3::RED;
			minBody = cg3::Point3d(_max.translation().x()+df, base.y(), base.z());
			maxBody = cg3::Point3d(_max.translation().x()+df*1.5, base.y(), base.z());
			arrowPoint = maxBody + cg3::Point3d(df/4,0,0);
			break;
		case HFBox::PLUS_Y :
			col = cg3::GREEN;
			minBody = cg3::Point3d(base.x(), _max.translation().y()+df, base.z());
			maxBody = cg3::Point3d(base.x(), _max.translation().y()+df*1.5 ,base.z());
			arrowPoint = maxBody + cg3::Point3d(0,df/4,0);
			break;
		case HFBox::PLUS_Z :
			col = cg3::BLUE;
			minBody = cg3::Point3d(base.x(), base.y(),_max.translation().z()+df);
			maxBody = cg3::Point3d(base.x(), base.y(),_max.translation().z()+df*1.5);
			arrowPoint = maxBody + cg3::Point3d(0,0,df/4);
			break;
		case HFBox::MINUS_X :
			col = cg3::RED;
			minBody = cg3::Point3d(_min.translation().x()-df, base.y(), base.z());
			maxBody = cg3::Point3d(_min.translation().x()-df*1.5, base.y(), base.z());
			arrowPoint = maxBody - cg3::Point3d(df/4,0,0);
			break;
		case HFBox::MINUS_Y :
			col = cg3::GREEN;
			minBody = cg3::Point3d(base.x(), _min.translation().y()-df, base.z());
			maxBody = cg3::Point3d(base.x(), _min.translation().y()-df*1.5 ,base.z());
			arrowPoint = maxBody - cg3::Point3d(0,df/4,0);
			break;
		case HFBox::MINUS_Z :
			col = cg3::BLUE;
			minBody = cg3::Point3d(base.x(), base.y(),_min.translation().z()-df);
			maxBody = cg3::Point3d(base.x(), base.y(),_min.translation().z()-df*1.5);
			arrowPoint = maxBody - cg3::Point3d(0,0,df/4);
			break;
		default:
			assert(0);
		}
		cg3::opengl::drawCylinder(minBody, maxBody, df/8, df/8, col);
		cg3::opengl::drawCylinder(maxBody, arrowPoint, df/5, 0, col);
	}
}

void ManipulableBoundingBox::checkIfGrabsMouse(int x, int y, const qglviewer::Camera * const camera)
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

void ManipulableBoundingBox::setDrawArrow(bool b)
{
	_drawArrow = b;
}

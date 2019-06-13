/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "manipulableboundingbox.h"

#include <cg3/utilities/const.h>
#include <cg3/viewer/opengl_objects/opengl_objects.h>

ManipulableBoundingBox::ManipulableBoundingBox() :
	df(1),
	millingDir(PLUS_Z)
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
	cg3::opengl::drawBox(_min.translation(), _max.translation(), cg3::BLACK);
	drawArrow();
}

cg3::Pointd ManipulableBoundingBox::sceneCenter() const
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
	cg3::opengl::drawBox(_min.translation(), _max.translation(), cg3::RED, 2);
	drawArrow();
}

void ManipulableBoundingBox::set(const cg3::Pointd &min, const cg3::Pointd &max)
{
	cg3::Pointd center = (min + max)*0.5;
	setTranslation(center);
	_min.setTranslation(min-center);
	_max.setTranslation(max-center);
	df = _min.translation().dist(_max.translation()) / 10;
	_min.setRadius(df/4);
	_max.setRadius(df/4);
}

void ManipulableBoundingBox::setMillingDirection(ManipulableBoundingBox::MillingDir dir)
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
	cg3::Pointd base = (_min.translation() + _max.translation()) * 0.5;
	cg3::Pointd minBody, maxBody, arrowPoint;
	cg3::Color col;
	switch (millingDir) {
	case PLUS_X :
		col = cg3::RED;
		minBody = cg3::Pointd(_max.translation().x()+df, base.y(), base.z());
		maxBody = cg3::Pointd(_max.translation().x()+df*1.5, base.y(), base.z());
		arrowPoint = maxBody + cg3::Pointd(df/4,0,0);
		break;
	case PLUS_Y :
		col = cg3::GREEN;
		minBody = cg3::Pointd(base.x(), _max.translation().y()+df, base.z());
		maxBody = cg3::Pointd(base.x(), _max.translation().y()+df*1.5 ,base.z());
		arrowPoint = maxBody + cg3::Pointd(0,df/4,0);
		break;
	case PLUS_Z :
		col = cg3::BLUE;
		minBody = cg3::Pointd(base.x(), base.y(),_max.translation().z()+df);
		maxBody = cg3::Pointd(base.x(), base.y(),_max.translation().z()+df*1.5);
		arrowPoint = maxBody + cg3::Pointd(0,0,df/4);
		break;
	case MINUS_X :
		col = cg3::RED;
		minBody = cg3::Pointd(_min.translation().x()-df, base.y(), base.z());
		maxBody = cg3::Pointd(_min.translation().x()-df*1.5, base.y(), base.z());
		arrowPoint = maxBody - cg3::Pointd(df/4,0,0);
		break;
	case MINUS_Y :
		col = cg3::GREEN;
		minBody = cg3::Pointd(base.x(), _min.translation().y()-df, base.z());
		maxBody = cg3::Pointd(base.x(), _min.translation().y()-df*1.5 ,base.z());
		arrowPoint = maxBody - cg3::Pointd(0,df/4,0);
		break;
	case MINUS_Z :
		col = cg3::BLUE;
		minBody = cg3::Pointd(base.x(), base.y(),_min.translation().z()-df);
		maxBody = cg3::Pointd(base.x(), base.y(),_min.translation().z()-df*1.5);
		arrowPoint = maxBody - cg3::Pointd(0,0,df/4);
		break;
	default:
		assert(0);
	}
	cg3::opengl::drawCylinder(minBody, maxBody, df/8, df/8, col);
	cg3::opengl::drawCylinder(maxBody, arrowPoint, df/5, 0, col);
}

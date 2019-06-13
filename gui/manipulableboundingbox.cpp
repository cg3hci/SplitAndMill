/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "manipulableboundingbox.h"

#include <cg3/utilities/const.h>
#include <cg3/viewer/opengl_objects/opengl_objects.h>

ManipulableBoundingBox::ManipulableBoundingBox()
{
	_min.setReferenceFrame(this);
	_max.setReferenceFrame(this);
	setWheelSensitivity(0);
	setGrabbingFactor(0.05);
}

void ManipulableBoundingBox::draw() const
{
	drawSpheres();
	cg3::opengl::drawBox(_min.translation(), _max.translation(), cg3::BLACK);
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
}

void ManipulableBoundingBox::set(const cg3::Pointd &min, const cg3::Pointd &max)
{
	cg3::Pointd center = (min + max)*0.5;
	setTranslation(center);
	_min.setTranslation(min-center);
	_max.setTranslation(max-center);
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

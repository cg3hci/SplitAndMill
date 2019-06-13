/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef MANIPULABLESPHERE_H
#define MANIPULABLESPHERE_H

#include <cg3/utilities/color.h>
#include <QGLViewer/manipulatedFrame.h>
#include <cg3/viewer/interfaces/drawable_object.h>
#include <cg3/viewer/interfaces/manipulable_object.h>

class ManipulableSphere : public virtual cg3::DrawableObject, public cg3::ManipulableObject
{
public:
	ManipulableSphere();

	void setRadius(double d);

	// DrawableObject interface
	void draw() const;
	void drawHighlighted() const;
	cg3::Pointd sceneCenter() const;
	double sceneRadius() const;


private:
	cg3::Pointd center;
	double radius;
	cg3::Color color;
	cg3::Color colorHighlited;
};

#endif // MANIPULABLESPHERE_H

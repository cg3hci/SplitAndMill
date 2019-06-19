/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef MANIPULABLEBOUNDINGBOX_H
#define MANIPULABLEBOUNDINGBOX_H

#include "manipulable_sphere.h"
#include <cg3/viewer/drawable_objects/drawable_bounding_box3.h>
#include <cg3/viewer/glcanvas.h>

#include "data_structures/hf_box.h"


class ManipulableBoundingBox : public cg3::ManipulableObject
{
public:
	ManipulableBoundingBox(cg3::viewer::GLCanvas& canvas);
	virtual ~ManipulableBoundingBox(){};

	// DrawableObject interface
	void draw() const;
	cg3::Point3d sceneCenter() const;
	double sceneRadius() const;

	// ManipulableObject interface
	void drawHighlighted() const;
	void checkIfGrabsMouse(int x, int y, const qglviewer::Camera * const camera);

	void setDrawArrow(bool b);

	void set(const cg3::Point3d &min, const cg3::Point3d &max);
	void setMillingDirection(HFBox::MillingDir dir);
	HFBox::MillingDir millingDirection() const {return millingDir;};
	cg3::Point3d min() const {return _min.position();}
	cg3::Point3d max() const {return _max.position();}

protected:
	void drawSpheres() const;
	void drawArrow() const;
	cg3::viewer::GLCanvas& canvas;
	bool _drawArrow;
	double df;
	ManipulableSphere _min, _max;
	HFBox::MillingDir millingDir;


};

#endif // MANIPULABLEBOUNDINGBOX_H

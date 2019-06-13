/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef MANIPULABLEBOUNDINGBOX_H
#define MANIPULABLEBOUNDINGBOX_H

#include "manipulablesphere.h"
#include <cg3/viewer/drawable_objects/drawable_bounding_box.h>

class ManipulableBoundingBox : public cg3::ManipulableObject
{
public:
	ManipulableBoundingBox();
	virtual ~ManipulableBoundingBox(){};

	// DrawableObject interface
	void draw() const;
	cg3::Pointd sceneCenter() const;
	double sceneRadius() const;

	// ManipulableObject interface
	void drawHighlighted() const;

	void set(const cg3::Pointd& min, const cg3::Pointd& max);
	const ManipulableSphere& min() const {return _min;}
	const ManipulableSphere& max() const {return _max;}

protected:
	void drawSpheres() const;
	ManipulableSphere _min, _max;
};

#endif // MANIPULABLEBOUNDINGBOX_H

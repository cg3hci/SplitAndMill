/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef GUIDES_H
#define GUIDES_H

#include <cg3/geometry/bounding_box3.h>
#include <cg3/viewer/interfaces/drawable_object.h>

class Guides : public cg3::DrawableObject
{
public:
	Guides();
	void clear();
	uint nGuides() const;
	void setBoundingBox(const cg3::BoundingBox3& bb);
	void pushGuide(const cg3::Point3d& p);
	void popGuide();
	cg3::Point3d nearest(const cg3::Point3d& p, uint dim) const;
	void setDrawX(bool b);
	void setDrawY(bool b);
	void setDrawZ(bool b);

	// DrawableObject interface
	void draw() const;
	cg3::Point3d sceneCenter() const;
	double sceneRadius() const;

private:
	cg3::BoundingBox3 bbox;
	std::vector<cg3::Point3d> points;
	bool drawX, drawY, drawZ;
};

#endif // GUIDES_H

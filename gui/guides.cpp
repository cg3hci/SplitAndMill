/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "guides.h"

#include <cg3/viewer/opengl_objects/opengl_objects3.h>
#include <cg3/utilities/const.h>

Guides::Guides() :
	bbox(cg3::Point3d(-1,-1,-1), cg3::Point3d(1,1,1)), drawX(false), drawY(false), drawZ(false)
{

}

void Guides::clear()
{
	points.clear();
}

uint Guides::nGuides() const
{
	return points.size();
}

void Guides::setBoundingBox(const cg3::BoundingBox3 &bb)
{
	bbox = bb;
}

void Guides::pushGuide(const cg3::Point3d &p)
{
	points.push_back(p);
}

void Guides::popGuide()
{
	points.pop_back();
}

cg3::Point3d Guides::nearest(const cg3::Point3d &p, uint dim) const
{
	assert(dim < 3);
	if (points.size() > 0){
		cg3::Point3d nearest = points[0];
		double minDist = std::abs(p(dim) - points[0](dim));
		for (uint i = 1; i < points.size(); ++i){
			double dist = std::abs(p(dim) - points[i](dim));
			if (dist < minDist){
				minDist = dist;
				nearest = points[i];
			}
		}
		return nearest;
	}
	return cg3::Point3d();
}

void Guides::setDrawX(bool b)
{
	drawX = b;
}

void Guides::setDrawY(bool b)
{
	drawY = b;
}

void Guides::setDrawZ(bool b)
{
	drawZ = b;
}

void Guides::draw() const
{
	for (const cg3::Point3d& p : points){
		if (drawX){
			cg3::opengl::drawDashedLine3(cg3::Point3d(bbox.minX(), p.y(), p.z()), cg3::Point3d(bbox.maxX(), p.y(), p.z()), cg3::BLACK, 1);
		}
		if (drawY){
			cg3::opengl::drawDashedLine3(cg3::Point3d(p.x(), bbox.minY(), p.z()), cg3::Point3d(p.x(), bbox.maxY(), p.z()), cg3::BLACK, 1);
		}
		if (drawZ){
			cg3::opengl::drawDashedLine3(cg3::Point3d(p.x(), p.y(), bbox.minZ()), cg3::Point3d(p.x(), p.y(), bbox.maxZ()), cg3::BLACK, 1);
		}
	}
}

cg3::Point3d Guides::sceneCenter() const
{
	return cg3::Point3d();
}

double Guides::sceneRadius() const
{
	return -1;
}

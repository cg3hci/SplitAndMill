/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "packing.h"

#include <cg3/cgal/minimum_bbox2.h>

namespace packing {

void rotateAllBlocks(const std::vector<HFBox> &boxes, std::vector<cg3::Dcel> &decomposition)
{
	for (unsigned int i = 0; i < boxes.size(); i++){
		cg3::Vec3d normal = cg3::AXIS[boxes[i].millingDirection()];
		normal.rotate(boxes[i].rotationMatrix().transpose());
		cg3::Vec3d axis = normal.cross(cg3::Z_AXIS);
		axis.normalize();
		double dot = normal.dot(cg3::Z_AXIS);
		double angle = acos(dot);

		Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
		if (normal != cg3::Z_AXIS){
			if (normal == -cg3::Z_AXIS){
				axis = cg3::Vec3d(1,0,0);
			}
			cg3::rotationMatrix(axis, angle, r);
		}
		decomposition[i].rotate(r);
		decomposition[i].updateBoundingBox();

		decomposition[i].translate(cg3::Point3d(0,0,-decomposition[i].boundingBox().min().z()));

		std::vector<cg3::Point2d> rect = cg3::cgal::minRectangle2D(decomposition[i]);

		cg3::Vec3d dir(rect[1].x() - rect[0].x(), rect[1].y() - rect[0].y(), 0);
		dir.normalize();
		angle = cg3::X_AXIS.dot(dir);

		//std::cerr << "Angle: " << angle << "\n";

		decomposition[i].rotate(cg3::Z_AXIS, angle);
		decomposition[i].updateBoundingBox();

		//std::cerr << "Area: " << _decomposition[i].boundingBox().lengthX() * _decomposition[i].boundingBox().lengthY() << "\n;
	}
}

double maxToolLengthBlock(const cg3::Dcel& block)
{
	double max = 0;
	for (const cg3::Dcel::HalfEdge* he : block.halfEdgeIterator()){
		double diff = std::abs(he->fromVertex()->coordinate().z() - he->toVertex()->coordinate().z());
		if (diff > max)
			max = diff;
	}
	return max;
}

/**
 * @brief worstBlockForToolLength -> find the most problematic block for a limited
 * tool length. Blocks must be rotated with basis parallel to Z plane
 * @param decomposition
 * @param length
 * @return the id of the worst block
 */
uint worstBlockForToolLength(const std::vector<cg3::Dcel> &decomposition, double& length)
{
	length = 0;
	uint worst = 0;
	uint i = 0;
	for (const cg3::Dcel& b : decomposition){
		double diff = maxToolLengthBlock(b);
		if (diff > length) {
			length = diff;
			worst = i;
		}
		++i;
	}
	return worst;
}

}

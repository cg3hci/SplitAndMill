/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef HF_BOX_H
#define HF_BOX_H

#include <cg3/geometry/bounding_box3.h>

class HFBox : public cg3::BoundingBox3
{
public:
	typedef enum {PLUS_X = 0, PLUS_Y, PLUS_Z, MINUS_X, MINUS_Y, MINUS_Z} MillingDir;
	HFBox();
	HFBox(const cg3::Point3d& min, const cg3::Point3d& max,
		  const cg3::Vec3& dir, const Eigen::Matrix3d& rot);
	HFBox(const cg3::Point3d& min, const cg3::Point3d& max,
		  uint dir, const Eigen::Matrix3d& rot);

	MillingDir millingDirection() const;
	Eigen::Matrix3d rotationMatrix() const;

private:
	MillingDir dir;
	Eigen::Matrix3d rot; //this is the matrix used to rotate the mesh!
						 //to rotate the box, use rot.transpose()
};

#endif // HF_BOX_H

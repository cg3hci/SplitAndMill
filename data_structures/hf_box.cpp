/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "hf_box.h"
#include <cg3/utilities/const.h>

HFBox::HFBox()
{

}

HFBox::HFBox(const cg3::Point3d &min, const cg3::Point3d &max, const cg3::Vec3d &dir, const Eigen::Matrix3d &rot) :
	cg3::BoundingBox3(min, max),
	rot(rot)
{
	bool found = false;
	for (uint i = 0; i < 6 && !found; ++i){
		if (dir == cg3::AXIS[i]){
			found = true;
			this->dir = (MillingDir)i;
		}
	}
	assert(found);
}

HFBox::HFBox(const cg3::Point3d &min, const cg3::Point3d &max, int dir, const Eigen::Matrix3d &rot) :
	cg3::BoundingBox3(min, max),
	dir((MillingDir)dir),
	rot(rot)
{
	assert(dir >= -1 && dir < 6);
}

HFBox::MillingDir HFBox::millingDirection() const
{
	return dir;
}

Eigen::Matrix3d HFBox::rotationMatrix() const
{
	return rot;
}

void HFBox::serialize(std::ofstream &binaryFile) const
{
	BoundingBox3::serialize(binaryFile);
	cg3::serializeObjectAttributes("HFBox", binaryFile, dir, rot);
}

void HFBox::deserialize(std::ifstream &binaryFile)
{
	BoundingBox3::deserialize(binaryFile);
	cg3::deserializeObjectAttributes("HFBox", binaryFile, dir, rot);
}

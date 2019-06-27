/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef PACKING_H
#define PACKING_H

#include "hf_box.h"
#include <cg3/meshes/dcel/dcel.h>

namespace packing {

void rotateAllBlocks(const std::vector<HFBox>& boxes, std::vector<cg3::Dcel>& decomposition);

uint worstBlockForToolLength(const std::vector<cg3::Dcel> &decomposition, double toolLength, double& factor);

uint worstBlockForStock(const std::vector<cg3::Dcel> &decomposition, const cg3::BoundingBox3& stock, double& factor);

std::vector< std::vector<std::pair<int, cg3::Point3d> > > packing(const std::vector<cg3::Dcel>& blocks, const cg3::BoundingBox3& stock, double distanceBetweenblocks);

}

#endif // PACKING_H

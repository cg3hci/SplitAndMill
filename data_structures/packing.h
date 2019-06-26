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

}

#endif // PACKING_H

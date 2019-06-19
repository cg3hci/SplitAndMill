/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef HIGH_FREQUENCIES_RESTORE_H
#define HIGH_FREQUENCIES_RESTORE_H

#include <cg3/meshes/dcel/dcel.h>

void restoreHighHrequenciesGaussSeidel(
		cg3::Dcel& smoothMesh,
		const cg3::Dcel& detailMesh,
		const std::vector<cg3::Vec3>& hfDirections,
		const int nIters,
		double flipAngle);

#endif // HIGH_FREQUENCIES_RESTORE_H

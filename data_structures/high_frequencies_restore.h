/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef HIGH_FREQUENCIES_RESTORE_H
#define HIGH_FREQUENCIES_RESTORE_H

#include <cg3/meshes/dcel/dcel.h>

bool validateMove(
		const cg3::Dcel::Vertex* v,
		cg3::Vec3d dir,
		const cg3::Point3d& vid_new_pos,
		double flipAngle);

std::vector<cg3::Vec3d> differentialCoordinates(const cg3::Dcel& m);

void restoreHighHrequenciesGaussSeidel(
		cg3::Dcel& smoothMesh,
		const cg3::Dcel& detailMesh,
		const std::vector<cg3::Vec3d>& hfDirections,
		const int nIters,
		double flipAngle);

#endif // HIGH_FREQUENCIES_RESTORE_H

/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef HIGH_FREQUENCIES_RESTORE_H
#define HIGH_FREQUENCIES_RESTORE_H

#include <cg3/meshes/dcel/dcel.h>
#include "data_structures/hf_box.h"

namespace restoreHF {

double det(
		const cg3::Point3d& p0,
		const cg3::Point3d& p1,
		const cg3::Point3d& p2,
		const cg3::Point3d& p3);

bool validateMove(
		const cg3::Dcel::Vertex* v,
		cg3::Vec3d dir,
		const cg3::Point3d& vid_new_pos,
		double flipAngle);

bool validateMove(
		const cg3::Dcel::Vertex* v,
		HFBox box,
		const cg3::Point3d& vid_new_pos,
		double flipAngle);

std::vector<cg3::Vec3d> differentialCoordinates(const cg3::Dcel& m);

void restoreHighHrequenciesGaussSeidelSingleIteration(
		cg3::Dcel &smoothMesh,
		const std::vector<cg3::Vec3d>& hfDirections,
		double flipAngle,
		const std::vector<cg3::Vec3d>& diffCoords);

void restoreHighHrequenciesGaussSeidel(
		cg3::Dcel& smoothMesh,
		const cg3::Dcel& detailMesh,
		const std::vector<cg3::Vec3d>& hfDirections,
		const int nIters,
		double flipAngle);

void restoreHighHrequenciesGaussSeidelSingleIteration(
		cg3::Dcel& smoothMesh,
		const std::vector<HFBox>& hfBoxes,
		double flipAngle,
		const std::vector<cg3::Vec3d> &diffCoords);

void restoreHighHrequenciesGaussSeidel(
		cg3::Dcel& smoothMesh,
		const cg3::Dcel& detailMesh,
		const std::vector<HFBox>& hfBoxes,
		const int nIters,
		double flipAngle);

}

#endif // HIGH_FREQUENCIES_RESTORE_H

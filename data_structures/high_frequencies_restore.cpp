/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "high_frequencies_restore.h"
#include <cg3/geometry/triangle3.h>
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>
#include <Eigen/Dense>

namespace restoreHF {

double det(
		const cg3::Point3d& p0,
		const cg3::Point3d& p1,
		const cg3::Point3d& p2,
		const cg3::Point3d& p3)
{
	Eigen::Matrix4d m;
	m << p0.x(), p0.y(), p0.z(), 1,
		 p1.x(), p1.y(), p1.z(), 1,
		 p2.x(), p2.y(), p2.z(), 1,
		 p3.x(), p3.y(), p3.z(), 1;

	return m.determinant();
}

bool validateMove(
		const cg3::Dcel::Vertex* v,
		cg3::Vec3d dir,
        const cg3::Point3d& vid_new_pos,
        double flipAngle)
{
	if (dir == cg3::Vec3d()) return true;
	for(const cg3::Dcel::Face* f : v->incidentFaceIterator()) {
		cg3::Triangle3d tri;
		tri[0] = f->vertex1() == v ? vid_new_pos : f->vertex1()->coordinate();
		tri[1] = f->vertex2() == v ? vid_new_pos : f->vertex2()->coordinate();
		tri[2] = f->vertex3() == v ? vid_new_pos : f->vertex3()->coordinate();

		cg3::Vec3d n = tri.normal();

		if (n.dot(dir) < flipAngle)
			return false;
	}
	return true;
}

bool validateMove(
		const cg3::Dcel::Vertex* v,
		HFBox box,
		const cg3::Point3d& vid_new_pos,
		double flipAngle)
{
	if (box.millingDirection() < 0) return true;
	Eigen::Matrix3d rot = box.rotationMatrix();
	cg3::Vec3d dir = cg3::AXIS[box.millingDirection()];
	dir.rotate(rot.transpose());

	cg3::SimpleEigenMesh mb = cg3::EigenMeshAlgorithms::makeBox(box);
	mb.rotate(rot.transpose());

	for(const cg3::Dcel::Face* f : v->incidentFaceIterator()) {
		cg3::Triangle3d tri;
		tri[0] = f->vertex1() == v ? vid_new_pos : f->vertex1()->coordinate();
		tri[1] = f->vertex2() == v ? vid_new_pos : f->vertex2()->coordinate();
		tri[2] = f->vertex3() == v ? vid_new_pos : f->vertex3()->coordinate();

		cg3::Vec3d n = tri.normal();

		if (n.dot(dir) < flipAngle)
			return false;

		//check if it is inside in the all 6 edges of the box
		double d0 = det(mb.vertex(6), mb.vertex(2), mb.vertex(1), vid_new_pos);
		double d1 = det(mb.vertex(6), mb.vertex(5), mb.vertex(4), vid_new_pos);
		double d2 = det(mb.vertex(6), mb.vertex(7), mb.vertex(3), vid_new_pos);
		double d3 = det(mb.vertex(3), mb.vertex(4), mb.vertex(0), vid_new_pos);
		double d4 = det(mb.vertex(0), mb.vertex(1), mb.vertex(2), vid_new_pos);
		double d5 = det(mb.vertex(0), mb.vertex(4), mb.vertex(1), vid_new_pos);

		if (d0 < -std::numeric_limits<double>::epsilon()) //+x
			return false;
		if (d1 < -std::numeric_limits<double>::epsilon()) //+y
			return false;
		if (d2 < -std::numeric_limits<double>::epsilon()) //+z
			return false;
		if (d3 < -std::numeric_limits<double>::epsilon()) //-x
			return false;
		if (d4 < -std::numeric_limits<double>::epsilon()) //-y
			return false;
		if (d5 < -std::numeric_limits<double>::epsilon()) //-z
			return false;

	}
	return true;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

std::vector<cg3::Vec3d> differentialCoordinates(const cg3::Dcel& m)
{
	std::vector<cg3::Vec3d> diffCoords;
	diffCoords.resize(m.numberVertices());

    #pragma omp parallel for
	for(uint vid=0; vid<m.numberVertices(); ++vid) {
		const cg3::Dcel::Vertex* v = m.vertex(vid);
		if (v != nullptr) {
			double w    = 1.0 / v->cardinality();
			cg3::Point3d  delta(0,0,0);
			for(const cg3::Dcel::Vertex* adj : v->adjacentVertexIterator()) {
				delta += w * (v->coordinate() - adj->coordinate());
			}
			diffCoords[vid] = delta;
		}
	}
	return diffCoords;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void restoreHighHrequenciesGaussSeidel(
        cg3::Dcel& smoothMesh,
        const cg3::Dcel& detailMesh,
		const std::vector<cg3::Vec3d>& hfDirections,
        const int nIters,
        double flipAngle)
{
	std::vector<cg3::Vec3d> diffCoords = differentialCoordinates(detailMesh);

	for(int i=0; i<nIters; ++i) {
		#pragma omp parallel for
		for(uint vid=0; vid<smoothMesh.numberVertices(); ++vid) {
			//std::cerr << vid << "\n";
			cg3::Dcel::Vertex* v = smoothMesh.vertex(vid);
			if (v != nullptr){
				cg3::Vec3d  gauss_iter(0,0,0);
				double w = 1.0 / v->cardinality();
				for(cg3::Dcel::Vertex* adj : v->adjacentVertexIterator()) {
					gauss_iter += w * adj->coordinate();
				}

				cg3::Point3d newPos = diffCoords.at(vid) + gauss_iter;

				// do binary search until the new pos does not violate the hf condition...
				int count = 0;
				while(!validateMove(v, hfDirections.at(vid), newPos, flipAngle) && ++count<5) {
					newPos = 0.5 * (newPos + v->coordinate());
				}

				if (count < 5){
					#pragma omp critical
					{
						v->setCoordinate(newPos);
						for (cg3::Dcel::Face* f : v->incidentFaceIterator())
							f->updateNormal();
					}
				}
			}
		}
	}
	smoothMesh.updateVertexNormals();
}

void restoreHighHrequenciesGaussSeidel(
		cg3::Dcel& smoothMesh,
		const cg3::Dcel& detailMesh,
		const std::vector<HFBox>& hfBoxes,
		const int nIters,
		double flipAngle)
{
	std::vector<cg3::Vec3d> diffCoords = differentialCoordinates(detailMesh);

	for(int i=0; i<nIters; ++i) {
		#pragma omp parallel for
		for(uint vid=0; vid<smoothMesh.numberVertices(); ++vid) {
			//std::cerr << vid << "\n";
			cg3::Dcel::Vertex* v = smoothMesh.vertex(vid);
			if (v != nullptr){
				cg3::Vec3d  gauss_iter(0,0,0);
				double w = 1.0 / v->cardinality();
				for(cg3::Dcel::Vertex* adj : v->adjacentVertexIterator()) {
					gauss_iter += w * adj->coordinate();
				}

				cg3::Point3d newPos = diffCoords.at(vid) + gauss_iter;

				// do binary search until the new pos does not violate the hf condition...
				int count = 0;
				while(!validateMove(v, hfBoxes.at(vid), newPos, flipAngle) && ++count<5) {
					newPos = 0.5 * (newPos + v->coordinate());
				}

				if (count < 5){
					#pragma omp critical
					{
						v->setCoordinate(newPos);
						for (cg3::Dcel::Face* f : v->incidentFaceIterator())
							f->updateNormal();
					}
				}
			}
		}
	}
	smoothMesh.updateVertexNormals();
}

}

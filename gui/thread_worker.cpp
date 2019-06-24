/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "thread_worker.h"

#include <cg3/utilities/timer.h>
#include <cg3/vcglib/smoothing.h>
#include <cg3/algorithms/global_optimal_rotation_matrix.h>
#include "data_structures/high_frequencies_restore.h"
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>
#include <cg3/libigl/booleans.h>


ThreadWorker::ThreadWorker()
{
}

void ThreadWorker::taubinSmoothing(cg3::Dcel mesh, uint nIt, double lambda, double mu)
{
	mesh = (cg3::Dcel)cg3::vcglib::taubinSmoothing(mesh, nIt, lambda, mu);
	emit taubinSmoothingCompleted(mesh);
}

void ThreadWorker::optimalOrientation(cg3::Dcel mesh, uint nDirs)
{
	std::vector<cg3::Vec3d> dirs = cg3::sphereCoverageFibonacci(nDirs-6);
	dirs.insert(dirs.end(), cg3::AXIS.begin(), cg3::AXIS.end());
	Eigen::Matrix3d rot = cg3::globalOptimalRotationMatrix(mesh, dirs);
	emit optimalOrientationCompleted(rot);
}

void ThreadWorker::restoreHighFrequencies(HFEngine* hfEngine, uint nIt, double flipAngle)
{
	cg3::Dcel smoothMesh = hfEngine->mesh();
	std::vector<cg3::Vec3d> hfDirections = hfEngine->restoreHighFrequenciesDirs();
	uint step = nIt / 10;

	std::vector<cg3::Vec3d> diffCoords = differentialCoordinates(hfEngine->originalMesh());


	for(uint i=0; i<nIt; ++i) {
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
		if (i % step == 0)
			emit setProgressBarValue(((double) i / nIt) * 100);
	}
	smoothMesh.updateVertexNormals();

	hfEngine->setMesh(smoothMesh);
	emit restoreHighFrequenciesCompleted();
}

void ThreadWorker::computeDecomposition(HFEngine *hfEngine)
{
	std::vector<cg3::Dcel> dec;

	uint i = 0;
	uint nBoxes = hfEngine->boxes().size();

	cg3::Dcel m = hfEngine->mesh();
	for (const HFBox& b : hfEngine->boxes()){
		cg3::SimpleEigenMesh box = cg3::EigenMeshAlgorithms::makeBox(b);
		box.rotate(b.rotationMatrix().transpose());
		dec.push_back(cg3::libigl::intersection(m, box));
		m = (cg3::Dcel)cg3::libigl::difference(m, box);

		emit setProgressBarValue(((double)i / nBoxes) * 100);
		i++;
	}

	emit computeDecompositionCompleted(dec);
}

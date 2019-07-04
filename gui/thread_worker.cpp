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
#include <cg3/algorithms/sphere_coverage.h>
#include "data_structures/high_frequencies_restore.h"
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>
#include <cg3/libigl/booleans.h>
#include "data_structures/packing.h"

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

void ThreadWorker::cut(cg3::Dcel mesh, HFBox hfbox)
{
	std::vector<uint> birthFaces;
	uint nFaces = mesh.numberFaces();

	cg3::BoundingBox3 bb(hfbox.min(), hfbox.max());
	cg3::SimpleEigenMesh b = cg3::EigenMeshAlgorithms::makeBox(bb);
	cg3::Dcel res =cg3::libigl::difference(mesh, b, birthFaces);
	for (cg3::Dcel::Face* f : res.faceIterator()){
		if (birthFaces[f->id()] < nFaces){
			f->setFlag(mesh.face(birthFaces[f->id()])->flag());
		}
		else{
			f->setFlag(1);
		}
	}
	emit cutCompleted(res);
}

void ThreadWorker::restoreHighFrequencies(HFEngine* hfEngine, uint nIt, double flipAngle)
{
	cg3::Dcel smoothMesh = hfEngine->mesh();
	std::vector<HFBox> hfBoxes = hfEngine->restoreHighFrequenciesBoxes();
	uint step = nIt / 10;

	std::vector<cg3::Vec3d> diffCoords = restoreHF::differentialCoordinates(hfEngine->originalMesh());

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
				while(!restoreHF::validateMove(v, hfBoxes.at(vid), newPos, flipAngle) && ++count<5) {
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
	Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(); //to avoid cast differences in comparison

	uint i = 0;
	uint nBoxes = hfEngine->boxes().size();

	cg3::SimpleEigenMesh m = hfEngine->mesh();
	for (const HFBox& b : hfEngine->boxes()){
		if (b.rotationMatrix() != rot){
			rot = b.rotationMatrix();
			m.rotate(rot);
		}

		cg3::SimpleEigenMesh box = cg3::EigenMeshAlgorithms::makeBox(b);
		//box.rotate(b.rotationMatrix().transpose());
		cg3::SimpleEigenMesh res = cg3::libigl::intersection(m, box);
		res.rotate(rot.transpose());
		dec.push_back(res);
		m = cg3::libigl::difference(m, box);

		emit setProgressBarValue(((double)i / nBoxes) * 100);
		i++;
	}

	emit computeDecompositionCompleted(dec);
}

void ThreadWorker::computeDecompositionExact(HFEngine* hfEngine){
	std::vector<cg3::Dcel> dec;
	Eigen::Matrix3d rtmp = Eigen::Matrix3d::Identity(); //to avoid cast differences in comparison
	cg3::libigl::CSGTree::MatrixX3E rot = Eigen::Matrix<cg3::libigl::CSGTree::ExactScalar,3,3>::Identity();

	uint i = 0;
	uint nBoxes = hfEngine->boxes().size();

	cg3::libigl::CSGTree tree = cg3::libigl::eigenMeshToCSGTree(hfEngine->mesh());
	for (const HFBox& b : hfEngine->boxes()){
		if (b.rotationMatrix() != rtmp){
			rtmp = b.rotationMatrix();
			rot = b.rotationMatrix().template cast<cg3::libigl::CSGTree::ExactScalar>();
			auto V = tree.V();
			for (unsigned int i = 0; i < V.rows(); i++){
				V.row(i) =  rot * V.row(i).transpose();
			}
			tree = cg3::libigl::CSGTree(V, tree.F());
		}

		cg3::SimpleEigenMesh box = cg3::EigenMeshAlgorithms::makeBox(b);
		//box.rotate(b.rotationMatrix().transpose());
		cg3::libigl::CSGTree treebox = cg3::libigl::eigenMeshToCSGTree(box);
		cg3::SimpleEigenMesh res = cg3::libigl::CSGTreeToEigenMesh(cg3::libigl::intersection(tree, treebox));
		res.rotate(rtmp.transpose());
		dec.push_back(res);
		tree = cg3::libigl::difference(tree, treebox);

		emit setProgressBarValue(((double)i / nBoxes) * 100);
		i++;
	}

	emit computeDecompositionCompleted(dec);
}

void ThreadWorker::packInOneStock(std::vector<cg3::Dcel> blocks, cg3::BoundingBox3 stock, double distanceBetweenBlocks)
{
	double factor = 1;
	double step = factor / 200;

	std::vector< std::vector<std::pair<int, cg3::Point3d> > > packs;

	do {
		std::vector<cg3::Dcel> bl = blocks;
		if (factor != 1){
			for (cg3::Dcel& b : bl){
				b.scale(factor);
				cg3::Vec3d dir = stock.min() - b.boundingBox().min();
				b.translate(dir);
			}
		}

		packs = packing::packing(bl, stock, distanceBetweenBlocks);

		factor-= step;
	} while(factor > 0 && packs.size() > 1);

	emit packInOneStockCompleted(factor > 0, factor, packs[0]);
}

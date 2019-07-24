/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "hf_engine_thread.h"
#include <cg3/vcglib/smoothing.h>
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>
#include <cg3/libigl/booleans.h>

#include "data_structures/hf_box.h"
#include "data_structures/high_frequencies_restore.h"

HFEngineThread::HFEngineThread()
{
}

HFEngineThread::HFEngineThread(const HFEngine &he) :
	HFEngine(he)
{
}

void HFEngineThread::taubinSmoothing(uint nIt, double lambda, double mu)
{
	HFEngine::taubinSmoothing(nIt, lambda, mu);
	emit taubinSmoothingCompleted();
}

Eigen::Matrix3d HFEngineThread::optimalOrientation(uint nDirs)
{
	Eigen::Matrix3d rot = HFEngine::optimalOrientation(nDirs);
	emit optimalOrientationCompleted(rot);
	return rot;
}

void HFEngineThread::cut(cg3::Dcel mesh, HFBox hfbox)
{
	std::vector<uint> birthFaces;
	uint nFaces = mesh.numberFaces();

	cg3::BoundingBox3 bb(hfbox.min(), hfbox.max());
	cg3::SimpleEigenMesh b = cg3::EigenMeshAlgorithms::makeBox(bb);
	cg3::Dcel block = cg3::libigl::intersection(mesh, b);
	//rotation of block....
	_tmpDecomposition.push_back(block);
	cg3::Dcel res =cg3::libigl::difference(mesh, b, birthFaces);
	_baseComplex = res;
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

void HFEngineThread::restoreHighFrequencies(uint nIterations, double flipAngle)
{
	uint step = nIterations / 10;
	std::vector<HFBox> boxes = restoreHighFrequenciesBoxes();

	std::vector<cg3::Vec3d> diffCoords = restoreHF::differentialCoordinates(_originalMesh);

	for(uint i=0; i<nIterations; ++i) {
		restoreHF::restoreHighHrequenciesGaussSeidelSingleIteration(
					_mesh,
					boxes,
					flipAngle,
					diffCoords);
		if (i % step == 0)
			emit setProgressBarValue(((double) i / nIterations) * 100);
	}
	_mesh.updateVertexNormals();

	emit restoreHighFrequenciesCompleted();
}

void HFEngineThread::computeDecomposition()
{
	_decomposition.clear();

	uint i = 0, r = 0;
	uint nBoxes = _boxes.size();

	cg3::SimpleEigenMesh m = _mesh;

	for (const HFBox& b : _boxes){
		while (r < rotHistory.size() && rotHistory[r].first <= i){
			m.rotate(rotHistory[r].second);
			++r;
		}

		cg3::SimpleEigenMesh box = cg3::EigenMeshAlgorithms::makeBox(b);
		cg3::SimpleEigenMesh res = cg3::libigl::intersection(m, box);
		for (int rr = r-1; rr >= 0; rr--) //order is important here
			res.rotate(rotHistory[rr].second.transpose());
		_decomposition.push_back(res);
		m = cg3::libigl::difference(m, box);

		emit setProgressBarValue(((double)i / nBoxes) * 100);
		i++;
	}

	colorDecomposition();
	emit computeDecompositionCompleted();
}

void HFEngineThread::computeDecompositionExact()
{
	cg3::libigl::CSGTree::MatrixX3E rot = Eigen::Matrix<cg3::libigl::CSGTree::ExactScalar,3,3>::Identity();

	uint i = 0, r = 0;
	uint nBoxes = _boxes.size();

	cg3::libigl::CSGTree tree = cg3::libigl::eigenMeshToCSGTree(_mesh);
	for (const HFBox& b : _boxes){
		while (r < rotHistory.size() && rotHistory[r].first <= i){
			rot = rotHistory[r].second.template cast<cg3::libigl::CSGTree::ExactScalar>();
			auto V = tree.V();
			for (unsigned int i = 0; i < V.rows(); i++){
				V.row(i) =  rot * V.row(i).transpose();
			}
			tree = cg3::libigl::CSGTree(V, tree.F());
			++r;
		}

		cg3::SimpleEigenMesh box = cg3::EigenMeshAlgorithms::makeBox(b);
		cg3::libigl::CSGTree treebox = cg3::libigl::eigenMeshToCSGTree(box);
		cg3::SimpleEigenMesh res = cg3::libigl::CSGTreeToEigenMesh(cg3::libigl::intersection(tree, treebox));
		for (int rr = r-1; rr >= 0; rr--) //order is important here
			res.rotate(rotHistory[rr].second.transpose());
		_decomposition.push_back(res);
		tree = cg3::libigl::difference(tree, treebox);

		emit setProgressBarValue(((double)i / nBoxes) * 100);
		i++;
	}

	colorDecomposition();
	emit computeDecompositionCompleted();
}

bool HFEngineThread::computeOneStockPackingFromDecomposition(
		const cg3::BoundingBox3& stock, double toolLength,
		cg3::Point2d frameThicknessStock, double zOffset,
		double distanceBetweenBlocks, cg3::Point2d clearnessStock,
		double clearnessTool, bool computeNegative)
{
	bool b = HFEngine::computeOneStockPackingFromDecomposition(stock, toolLength, frameThicknessStock, zOffset, distanceBetweenBlocks, clearnessStock, clearnessTool, computeNegative);
	emit computeOneStockPackingFromDecompositionCompleted(b);
	return b;
}

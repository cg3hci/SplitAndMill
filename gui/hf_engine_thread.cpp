﻿/*
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
	Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(); //to avoid cast differences in comparison

	uint i = 0;
	uint nBoxes = _boxes.size();

	cg3::SimpleEigenMesh m = _mesh;
	for (const HFBox& b : _boxes){
		if (b.rotationMatrix() != rot){
			rot = b.rotationMatrix();
			m.rotate(rot);
		}

		cg3::SimpleEigenMesh box = cg3::EigenMeshAlgorithms::makeBox(b);
		//box.rotate(b.rotationMatrix().transpose());
		cg3::SimpleEigenMesh res = cg3::libigl::intersection(m, box);
		res.rotate(rot.transpose());
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
	Eigen::Matrix3d rtmp = Eigen::Matrix3d::Identity(); //to avoid cast differences in comparison
	cg3::libigl::CSGTree::MatrixX3E rot = Eigen::Matrix<cg3::libigl::CSGTree::ExactScalar,3,3>::Identity();

	uint i = 0;
	uint nBoxes = _boxes.size();

	cg3::libigl::CSGTree tree = cg3::libigl::eigenMeshToCSGTree(_mesh);
	for (const HFBox& b : _boxes){
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
		_decomposition.push_back(res);
		tree = cg3::libigl::difference(tree, treebox);

		emit setProgressBarValue(((double)i / nBoxes) * 100);
		i++;
	}

	colorDecomposition();
	emit computeDecompositionCompleted();
}

bool HFEngineThread::computeOneStockPackingFromDecomposition(const cg3::BoundingBox3 &stock, double toolLength, double distanceBetweenBlocks, cg3::Point2d clearnessStock, double clearnessTool)
{
	bool b = HFEngine::computeOneStockPackingFromDecomposition(stock, toolLength, distanceBetweenBlocks, clearnessStock, clearnessTool);
	emit computeOneStockPackingFromDecompositionCompleted(b);
	return b;
}
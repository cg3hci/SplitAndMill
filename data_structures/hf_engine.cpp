/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "hf_engine.h"

#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>
#include <cg3/libigl/booleans.h>
#include <cg3/libigl/mesh_distance.h>
#include <cg3/cgal/aabb_tree3.h>

#include "high_frequencies_restore.h"
#include "packing.h"

HFEngine::HFEngine() :
	useSmoothedMesh(false)
{
}

void HFEngine::clear()
{
	useSmoothedMesh = false;
	_boxes.clear();
}

void HFEngine::setMesh(const cg3::Dcel &mesh)
{
	this->_mesh = mesh;
}

void HFEngine::setOriginalMesh(const cg3::Dcel &mesh)
{
	_originalMesh = mesh;
	useSmoothedMesh = true;
}

void HFEngine::setUseSmoothedMesh(bool b)
{
	useSmoothedMesh = b;
}

bool HFEngine::usesSmoothedMesh() const
{
	return useSmoothedMesh;
}

void HFEngine::pushBox(const HFBox &box)
{
	_boxes.push_back(box);
}

void HFEngine::popBox()
{
	_boxes.pop_back();
}

const std::vector<HFBox>& HFEngine::boxes() const
{
	return _boxes;
}

std::vector<cg3::Vec3d> HFEngine::restoreHighFrequenciesDirs() const
{
	Eigen::Matrix3d actualRot = Eigen::Matrix3d::Identity();
	cg3::Dcel m = _mesh;
	cg3::cgal::AABBTree3 tree(m);
	std::vector<cg3::Vec3d> dirs(m.numberVertices(), cg3::Vec3d());
	std::vector<bool> visited(m.numberVertices(), false);

	for (const HFBox& b : _boxes){
		if (b.rotationMatrix() != actualRot){
			actualRot = b.rotationMatrix();
			m = _mesh;
			m.rotate(actualRot);
			tree = cg3::cgal::AABBTree3(m);
		}

		std::list<const cg3::Dcel::Face*> list = tree.containedDcelFaces(b);

		for (const cg3::Dcel::Face* f : list){
			for (const cg3::Dcel::Vertex* v : f->incidentVertexIterator()){
				if (!visited[v->id()] && b.isInside(v->coordinate())){
					visited[v->id()] = true;
					cg3::Vec3d dir = cg3::AXIS[b.millingDirection()];
					dir.rotate(actualRot.transpose());
					dirs[v->id()] = dir;
				}
			}
		}
	}

	return dirs;
}

void HFEngine::restoreHighFrequencies(uint nIterations, double flipAngle)
{
	std::vector<cg3::Vec3d> dirs = restoreHighFrequenciesDirs();

	restoreHF::restoreHighHrequenciesGaussSeidel(_mesh, _originalMesh, dirs, nIterations, flipAngle);
}

double HFEngine::hausdorffDistance() const
{
	return cg3::libigl::hausdorffDistance(_mesh, _originalMesh);
}

void HFEngine::computeDecomposition()
{
	_decomposition.clear();
	cg3::Dcel m = _mesh;
	for (const HFBox& b : _boxes){
		cg3::SimpleEigenMesh box = cg3::EigenMeshAlgorithms::makeBox(b);
		box.rotate(b.rotationMatrix().transpose());
		//box.saveOnObj("debug/b" + std::to_string(i++) + ".obj");
		_decomposition.push_back(cg3::libigl::intersection(m, box));
		m = (cg3::Dcel)cg3::libigl::difference(m, box);
	}
}

std::vector<cg3::Dcel> HFEngine::decomposition() const
{
	return _decomposition;
}

std::vector<cg3::Dcel> &HFEngine::decomposition()
{
	return _decomposition;
}

void HFEngine::rotateAllBlocks()
{
	_packing.resize(1);
	_packing[0] = _decomposition;
	packing::rotateAllBlocks(_boxes, _packing[0]);
}

std::vector<std::vector<cg3::Dcel>> HFEngine::packing() const
{
	return _packing;
}

std::vector<std::vector<cg3::Dcel>> &HFEngine::packing()
{
	return _packing;
}

cg3::Dcel HFEngine::mesh() const
{
	return _mesh;
}

cg3::Dcel HFEngine::originalMesh() const
{
	return _originalMesh;
}

void HFEngine::serialize(std::ofstream &binaryFile) const
{
	cg3::serializeObjectAttributes("HFEngine", binaryFile, _mesh, _originalMesh, useSmoothedMesh, _boxes, _decomposition, _packing);
}

void HFEngine::deserialize(std::ifstream &binaryFile)
{
	cg3::deserializeObjectAttributes("HFEngine", binaryFile, _mesh, _originalMesh, useSmoothedMesh, _boxes, _decomposition, _packing);
}

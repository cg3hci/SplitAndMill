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

#include "lib/packing/binpack2d.h"

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

std::vector<cg3::Dcel> HFEngine::packingPreProcessing(const cg3::BoundingBox3& stock, double toolLength, cg3::Point2d clearnessStock, double clearnessTool, double& factor)
{
	//rotation
	std::vector<cg3::Dcel> tmpPacking = _decomposition;
	packing::rotateAllBlocks(_boxes, tmpPacking);

	//scaling
	cg3::BoundingBox3 actualStock(stock.min(), cg3::Point3d(stock.maxX()-clearnessStock.x(), stock.maxY()-clearnessStock.x(), stock.maxZ()-clearnessStock.y()));
	toolLength -= clearnessTool;

	double sizesFactor, toolFactor;
	packing::worstBlockForStock(tmpPacking, actualStock, sizesFactor);
	packing::worstBlockForToolLength(tmpPacking, toolLength, toolFactor);

	factor = sizesFactor < toolFactor ? sizesFactor : toolFactor;

	for (cg3::Dcel& b : tmpPacking){
		b.scale(factor);
		cg3::Vec3d dir = stock.min() - b.boundingBox().min();
		b.translate(dir);
	}
	return tmpPacking;
}

void HFEngine::comutePackingFromDecomposition(const cg3::BoundingBox3& stock, double toolLength, double distanceBetweenblocks, cg3::Point2d clearnessStock, double clearnessTool)
{
	_packing.clear();

	double factor;
	std::vector<cg3::Dcel> tmpPacking = packingPreProcessing(stock, toolLength, clearnessStock, clearnessTool, factor);

	//packing
	std::vector< std::vector<std::pair<int, cg3::Point3d> > > packs =
			packing::packing(tmpPacking, stock, distanceBetweenblocks);

	uint i = 0;
	for (const auto& pack : packs){
		_packing.push_back(std::vector<cg3::Dcel>(pack.size()));
		uint j = 0;
		for (const std::pair<int, cg3::Point3d>& p : pack){
			bool rotated = false;
			if (p.first < 0)
				rotated = true;

			_packing[i][j] = tmpPacking[std::abs(p.first)-1];
			if (rotated) {
				_packing[i][j].rotate(cg3::Z_AXIS, M_PI/2);
				_packing[i][j].translate(stock.min() - _packing[i][j].boundingBox().min());
			}
			_packing[i][j].translate(p.second - stock.min());
			j++;
		}
		i++;
	}

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

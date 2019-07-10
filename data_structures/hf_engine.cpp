/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "hf_engine.h"

#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>
#include <cg3/algorithms/global_optimal_rotation_matrix.h>
#include <cg3/algorithms/sphere_coverage.h>
#include <cg3/libigl/booleans.h>
#include <cg3/libigl/mesh_distance.h>
#include <cg3/cgal/aabb_tree3.h>
#include <cg3/vcglib/smoothing.h>


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
	_decomposition.clear();
	_packing.clear();
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

Eigen::Matrix3d HFEngine::optimalOrientation(uint nDirs)
{
	std::vector<cg3::Vec3d> dirs = cg3::sphereCoverageFibonacci(nDirs-6);
	dirs.insert(dirs.end(), cg3::AXIS.begin(), cg3::AXIS.end());
	Eigen::Matrix3d rot = cg3::globalOptimalRotationMatrix(_mesh, dirs);
	return rot;
}

void HFEngine::taubinSmoothing(uint nIt, double lambda, double mu)
{
	if (!useSmoothedMesh){
		_originalMesh = _mesh;
	}
	_mesh = (cg3::Dcel)cg3::vcglib::taubinSmoothing(_mesh, nIt, lambda, mu);
	useSmoothedMesh = true;
}

bool HFEngine::usesSmoothedMesh() const
{
	return useSmoothedMesh;
}

void HFEngine::pushRotation(const Eigen::Matrix3d &rot)
{
	rotHistory.push_back(std::make_pair(_boxes.size(), rot));
}

void HFEngine::pushRotation(uint nBoxes, const Eigen::Matrix3d &rot)
{
	rotHistory.push_back(std::make_pair(nBoxes, rot));
}

void HFEngine::popRotation()
{
	rotHistory.pop_back();
}

const std::vector<std::pair<uint, Eigen::Matrix3d> > &HFEngine::rotationHistory()
{
	return rotHistory;
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

void HFEngine::restoreHighFrequencies(uint nIterations, double flipAngle)
{
	std::vector<HFBox> boxes = restoreHighFrequenciesBoxes();

	restoreHF::restoreHighHrequenciesGaussSeidel(_mesh, _originalMesh, boxes, nIterations, flipAngle);
}

double HFEngine::hausdorffDistance() const
{
	return cg3::libigl::hausdorffDistance(_mesh, _originalMesh);
}

void HFEngine::computeDecomposition()
{
	_decomposition.clear();

	uint i = 0, r = 0;

	cg3::SimpleEigenMesh m = _mesh;

	for (const HFBox& b : _boxes){
		while (r < rotHistory.size() && rotHistory[r].first <= i){
			m.rotate(rotHistory[r].second);
			++r;
		}

		cg3::SimpleEigenMesh box = cg3::EigenMeshAlgorithms::makeBox(b);
		cg3::SimpleEigenMesh res = cg3::libigl::intersection(m, box);
		for (int rr = r-1; rr >= 0; rr--)
			res.rotate(rotHistory[rr].second.transpose());
		_decomposition.push_back(res);
		m = cg3::libigl::difference(m, box);

		i++;
	}

	colorDecomposition();
}

void HFEngine::computeDecompositionExact()
{
	cg3::libigl::CSGTree::MatrixX3E rot = Eigen::Matrix<cg3::libigl::CSGTree::ExactScalar,3,3>::Identity();

	uint i = 0, r = 0;

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
		for (int rr = r-1; rr >= 0; rr--)
			res.rotate(rotHistory[rr].second.transpose());
		_decomposition.push_back(res);
		tree = cg3::libigl::difference(tree, treebox);

		i++;
	}

	colorDecomposition();
}

void HFEngine::colorDecomposition()
{
	constexpr int nColors = 9;
	std::array<QColor, nColors> colors;
	colors[0] = QColor(221, 126, 107); //
	colors[1] = QColor(255, 229, 153); //
	colors[2] = QColor(164, 194, 244); //
	colors[3] = QColor(213, 166, 189); //
	colors[4] = QColor(234, 153, 153); //
	colors[5] = QColor(182, 215, 168); //
	colors[6] = QColor(249, 203, 156);//
	colors[7] = QColor(180, 167, 214);//
	colors[8] = QColor(162, 196, 201);

	cg3::cgal::AABBTree3 tree(_mesh);

	std::map< const cg3::Dcel::Vertex*, int > mapping = mappingVertexToBlock(tree);
	std::vector< std::set<int> > adjacences(_decomposition.size());
	for (const cg3::Dcel::Vertex* v : _mesh.vertexIterator()){
		if (mapping.find(v) != mapping.end()){
			int hev = mapping[v];
			for (const cg3::Dcel::Vertex* adj : v->adjacentVertexIterator()){
				if (mapping.find(adj) != mapping.end()){
					int headj = mapping[adj];
					if (hev != headj){
						adjacences[hev].insert(headj);
						adjacences[headj].insert(hev);
					}
				}
			}
		}
	}

	std::vector<bool> colored(_decomposition.size(), false);
	std::vector<cg3::Color> heColors(_decomposition.size(), cg3::Color(0,0,0));
	for (unsigned int i = 0; i < _decomposition.size(); i++){
		if (!colored[i]){
			std::set<cg3::Color> adjColors;
			for (int adj : adjacences[i]){
				if (colored[adj])
					adjColors.insert(heColors[adj]);
			}

			cg3::Color color;
			bool finded = false;
			unsigned int k = i % nColors;
			do {
				if (adjColors.find(colors[k]) == adjColors.end()){
					finded = true;
					color = colors[k];
				}
				k = (k+1)%nColors;
			} while(k != i %nColors && !finded);

			if (finded)
				heColors[i] = color;
			else
				heColors[i] = cg3::Color(0,0,0);
			colored[i] = true;

			_decomposition[i].setFaceColors(color);
		}
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

void HFEngine::computePackingFromDecomposition(const cg3::BoundingBox3& stock, double toolLength, double distanceBetweenblocks, cg3::Point2d clearnessStock, double clearnessTool, double factor)
{
	_packing.clear();

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

bool HFEngine::computeOneStockPackingFromDecomposition(const cg3::BoundingBox3 &stock, double toolLength, double distanceBetweenBlocks, cg3::Point2d clearnessStock, double clearnessTool)
{
	auto tmpPacking = packingPreProcessing(stock, toolLength, clearnessStock, clearnessTool);

	double factor = 1;
	double step = factor / 200;
	std::vector< std::vector<std::pair<int, cg3::Point3d> > > packs;
	do {
		std::vector<cg3::Dcel> bl = tmpPacking;
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

	if (factor > 0){
		_packing.clear();
		setOneStockPacking(tmpPacking, factor, stock, packs[0]);
	}
	return factor > 0;
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
	cg3::serialize(rotHistory, binaryFile);
}

void HFEngine::deserialize(std::ifstream &binaryFile)
{
	cg3::deserializeObjectAttributes("HFEngine", binaryFile, _mesh, _originalMesh, useSmoothedMesh, _boxes, _decomposition, _packing);

	try {
		cg3::deserialize(rotHistory, binaryFile);
	} catch (std::ios_base::failure&) {

	}
}

std::vector<HFBox> HFEngine::restoreHighFrequenciesBoxes() const
{
	Eigen::Matrix3d actualRot = Eigen::Matrix3d::Identity();
	cg3::Dcel m = _mesh;
	cg3::cgal::AABBTree3 tree(m);
	HFBox nullBox(m.boundingBox().min(), m.boundingBox().max(), -1, Eigen::Matrix3d::Identity());

	std::vector<HFBox> boxes(m.numberVertices(), nullBox);
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
					boxes[v->id()] = b;
				}
			}
		}
	}
	return boxes;
}

std::map< const cg3::Dcel::Vertex*, int > HFEngine::mappingVertexToBlock(const cg3::cgal::AABBTree3& tree){
	std::map< const cg3::Dcel::Vertex*,int > mapping;

	for (uint i = 0; i < _decomposition.size(); i++){
		for (cg3::Dcel::Vertex* vb : _decomposition[i].vertexIterator()){
			const cg3::Dcel::Vertex* v = tree.nearestDcelVertex(vb->coordinate());
			assert(v != nullptr);
			if (v->coordinate().dist(vb->coordinate()) < cg3::EPSILON){
				mapping[v] = i;
			}
			if (tree.squaredDistance(vb->coordinate() < cg3::EPSILON)){
				vb->setNormal(v->normal());
			}
		}
	}
	return mapping;
}

std::vector<cg3::Dcel> HFEngine::rotateAllBlocks()
{
	std::vector<cg3::Dcel> decomposition = _decomposition;
	for (unsigned int i = 0; i < _boxes.size(); i++){
		cg3::Vec3d normal = cg3::AXIS[_boxes[i].millingDirection()];
		for (uint k = 0; rotHistory[k].first <= i; ++k){
			normal.rotate(rotHistory[k].second.transpose());
			normal.normalize();
		}
		//normal.rotate(_boxes[i].rotationMatrix().transpose());
		cg3::Vec3d axis = normal.cross(cg3::Z_AXIS);
		axis.normalize();
		double dot = normal.dot(cg3::Z_AXIS);
		double angle = acos(dot);

		Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
		if (normal != cg3::Z_AXIS){
			if (normal == -cg3::Z_AXIS){
				axis = cg3::Vec3d(1,0,0);
			}
			cg3::rotationMatrix(axis, angle, r);
		}
		decomposition[i].rotate(r);
		decomposition[i].updateBoundingBox();

		decomposition[i].translate(cg3::Point3d(0,0,-decomposition[i].boundingBox().min().z()));
	}
	return decomposition;
}

std::vector<cg3::Dcel> HFEngine::packingPreProcessing(const cg3::BoundingBox3& stock, double toolLength, cg3::Point2d clearnessStock, double clearnessTool, double factor)
{
	//rotation
	std::vector<cg3::Dcel> tmpPacking = rotateAllBlocks();

	//scaling
	cg3::BoundingBox3 actualStock(stock.min(), cg3::Point3d(stock.maxX()-clearnessStock.x(), stock.maxY()-clearnessStock.x(), stock.maxZ()-clearnessStock.y()));
	toolLength -= clearnessTool;

	double sizesFactor, toolFactor;
	packing::worstBlockForStock(tmpPacking, actualStock, sizesFactor);
	packing::worstBlockForToolLength(tmpPacking, toolLength, toolFactor);

	factor = (sizesFactor < toolFactor ? sizesFactor : toolFactor) * factor;

	for (cg3::Dcel& b : tmpPacking){
		b.scale(factor);
		cg3::Vec3d dir = stock.min() - b.boundingBox().min();
		b.translate(dir);
	}
	return tmpPacking;
}

void HFEngine::setOneStockPacking(std::vector<cg3::Dcel>tmpPacking, double factor, const cg3::BoundingBox3& stock, const std::vector<std::pair<int, cg3::Point3d>>& pack)
{
	_packing.clear();

	for (cg3::Dcel& b : tmpPacking){
		b.scale(factor);
		cg3::Vec3d dir = stock.min() - b.boundingBox().min();
		b.translate(dir);
	}

	_packing.push_back(std::vector<cg3::Dcel>(pack.size()));
	uint j = 0;
	for (const std::pair<int, cg3::Point3d>& p : pack){
		bool rotated = false;
		if (p.first < 0)
			rotated = true;

		_packing[0][j] = tmpPacking[std::abs(p.first)-1];
		if (rotated) {
			_packing[0][j].rotate(cg3::Z_AXIS, M_PI/2);
			_packing[0][j].translate(stock.min() - _packing[0][j].boundingBox().min());
		}
		_packing[0][j].translate(p.second - stock.min());
		j++;
	}
}

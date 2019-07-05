/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef HF_ENGINE_H
#define HF_ENGINE_H

#include <cg3/meshes/dcel/dcel.h>
#include <cg3/cgal/aabb_tree3.h>
#include "hf_box.h"

class HFEngine : public cg3::SerializableObject
{
public:
	HFEngine();
	void clear();
	void setMesh(const cg3::Dcel& _mesh);
	void setOriginalMesh(const cg3::Dcel& _mesh);
	void setUseSmoothedMesh(bool b);
	bool usesSmoothedMesh() const;
	void pushBox(const HFBox& box);
	void popBox();
	const std::vector<HFBox>& boxes() const;
	std::vector<cg3::Vec3d> restoreHighFrequenciesDirs() const;
	std::vector<HFBox> restoreHighFrequenciesBoxes() const;
	void restoreHighFrequencies(uint nIterations, double flipAngle);
	double hausdorffDistance() const;
	void computeDecomposition();
	void colorDecomposition();
	std::vector<cg3::Dcel> decomposition() const;
	std::vector<cg3::Dcel>& decomposition();

	std::vector<cg3::Dcel> packingPreProcessing(const cg3::BoundingBox3& stock, double toolLength, cg3::Point2d clearnessStock, double clearnessTool, double factor = 1);
	void comutePackingFromDecomposition(const cg3::BoundingBox3 &stock, double toolLength, double distangeBetweenblocks = 5, cg3::Point2d clearnessStock = cg3::Point2d(5, 2), double clearnessTool = 1, double factor = 1);
	void setOneStockPacking(std::vector<cg3::Dcel>tmpPacking, double factor, const cg3::BoundingBox3 &stock, const std::vector<std::pair<int, cg3::Point3d> > &pack);
	void computeOneStockPackingFromDecomposition(const cg3::BoundingBox3 &stock, double toolLength, double distangeBetweenblocks = 5, cg3::Point2d clearnessStock = cg3::Point2d(5, 2), double clearnessTool = 1);

	std::vector<std::vector<cg3::Dcel> > packing() const;
	std::vector<std::vector<cg3::Dcel> >& packing();

	cg3::Dcel mesh() const;
	cg3::Dcel originalMesh() const;

	// SerializableObject interface
	void serialize(std::ofstream &binaryFile) const;
	void deserialize(std::ifstream &binaryFile);

private:
	cg3::Dcel _mesh;
	cg3::Dcel _originalMesh;
	bool useSmoothedMesh;
	std::vector<HFBox> _boxes;
	std::vector<cg3::Dcel> _decomposition;
	std::vector<std::vector<cg3::Dcel>> _packing;
	std::map<const cg3::Dcel::Vertex *, int> mappingVertexToBlock(const cg3::cgal::AABBTree3 &tree);
};

#endif // HF_ENGINE_H

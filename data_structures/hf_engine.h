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
	virtual Eigen::Matrix3d optimalOrientation(uint nDirs);
	virtual void taubinSmoothing(uint nIt, double lambda, double mu);
	bool usesSmoothedMesh() const;
	void pushRotation(const Eigen::Matrix3d& rot);
	void pushRotation(uint nBoxes, const Eigen::Matrix3d& rot);
	void popRotation();
	const std::vector<std::pair<uint, Eigen::Matrix3d>>& rotationHistory();
	void pushBox(const HFBox& box);
	void popBox();
	const std::vector<HFBox>& boxes() const;
	virtual void restoreHighFrequencies(uint nIterations, double flipAngle);
	double hausdorffDistance() const;
	virtual void computeDecomposition();
	virtual void computeDecompositionExact();
	void colorDecomposition();

	const std::vector<cg3::Dcel>& tmpDecomposition() const;
	std::vector<cg3::Dcel>& tmpDecomposition();
	const cg3::Dcel& baseComplex() const;
	cg3::Dcel& baseComplex();

	const std::vector<cg3::Dcel>& decomposition() const;
	std::vector<cg3::Dcel>& decomposition();


	virtual void computePackingFromDecomposition(
			const cg3::BoundingBox3 &stock, double toolLength,
			const cg3::Point2d& frameThicknessStock, double zOffset,
			double distangeBetweenblocks = 5, cg3::Point2d clearnessStock = cg3::Point2d(5, 2),
			double clearnessTool = 1, double factor = 1, bool computeNegative = false);

	virtual bool computeOneStockPackingFromDecomposition(const cg3::BoundingBox3 &stock, double toolLength,
			cg3::Point2d frameThicknessStock, double zOffset,
			double distanceBetweenBlocks = 5, cg3::Point2d clearnessStock = cg3::Point2d(5, 2),
			double clearnessTool = 1, bool computeNegative = false);

	cg3::Dcel computeFrameStock(
			const cg3::BoundingBox3 &stock, cg3::Point2d frameThicknessStock) const;

	std::vector<std::vector<cg3::Dcel> > packing() const;
	std::vector<std::vector<cg3::Dcel> >& packing();

	cg3::Dcel mesh() const;
	cg3::Dcel originalMesh() const;

	// SerializableObject interface
	void serialize(std::ofstream &binaryFile) const;
	void deserialize(std::ifstream &binaryFile);

protected:
	cg3::Dcel _mesh;
	cg3::Dcel _originalMesh;
	bool useSmoothedMesh;
	std::vector<std::pair<uint, Eigen::Matrix3d>> rotHistory;
	//history of rotations here
	std::vector<HFBox> _boxes;
	std::vector<cg3::Dcel> _tmpDecomposition;
	cg3::Dcel _baseComplex;
	std::vector<cg3::Dcel> _decomposition;
	std::vector<std::vector<cg3::Dcel>> _packing;
	std::vector<HFBox> restoreHighFrequenciesBoxes() const;
	std::map<const cg3::Dcel::Vertex *, int> mappingVertexToBlock(const cg3::cgal::AABBTree3 &tree);
	std::vector<cg3::Dcel> rotateAllBlocks();
	std::vector<cg3::Dcel> packingPreProcessing(const cg3::BoundingBox3& stock, double toolLength, cg3::Point2d clearnessStock, double clearnessTool, double factor = 1);
	void setOneStockPacking(std::vector<cg3::Dcel>tmpPacking, double factor, const cg3::BoundingBox3 &stock, const std::vector<std::pair<int, cg3::Point3d> > &pack);
};

#endif // HF_ENGINE_H

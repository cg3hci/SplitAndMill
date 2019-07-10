/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "packing.h"

#include <cg3/cgal/minimum_bbox2.h>
#include "lib/packing/binpack2d.h"

namespace packing {

void rotateAllBlocks(const std::vector<HFBox> &boxes, std::vector<cg3::Dcel> &decomposition)
{
	for (unsigned int i = 0; i < boxes.size(); i++){
		cg3::Vec3d normal = cg3::AXIS[boxes[i].millingDirection()];
		normal.rotate(boxes[i].rotationMatrix().transpose());
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

		/*std::vector<cg3::Point2d> rect = cg3::cgal::minRectangle2D(decomposition[i]);

		cg3::Vec3d dir(rect[1].x() - rect[0].x(), rect[1].y() - rect[0].y(), 0);
		dir.normalize();
		angle = cg3::X_AXIS.dot(dir);

		//std::cerr << "Angle: " << angle << "\n";

		decomposition[i].rotate(cg3::Z_AXIS, angle);
		decomposition[i].updateBoundingBox();*/

		//std::cerr << "Area: " << _decomposition[i].boundingBox().lengthX() * _decomposition[i].boundingBox().lengthY() << "\n;
	}
}

double maxToolLengthBlock(const cg3::Dcel& block)
{
	double max = 0;
	for (const cg3::Dcel::HalfEdge* he : block.halfEdgeIterator()){
		double diff = std::abs(he->fromVertex()->coordinate().z() - he->toVertex()->coordinate().z());
		if (diff > max)
			max = diff;
	}
	return max;
}

/**
 * @brief worstBlockForToolLength -> find the most problematic block for a limited
 * tool length. Blocks must be rotated with basis parallel to Z plane
 * @param decomposition
 * @param length
 * @return the id of the worst block
 */
uint worstBlockForToolLength(const std::vector<cg3::Dcel> &decomposition, double toolLength, double& factor)
{
	double length = 0;
	uint worst = 0;
	uint i = 0;
	for (const cg3::Dcel& b : decomposition){
		double diff = maxToolLengthBlock(b);
		if (diff > length) {
			length = diff;
			worst = i;
		}
		++i;
	}
	factor = toolLength / length;
	return worst;
}

void minMaxEdge(const cg3::BoundingBox3& bb, double& min, double& max){
	if (bb.lengthX() < bb.lengthY()){
		min = bb.lengthX();
		max = bb.lengthY();
	}
	else {
		min = bb.lengthY();
		max = bb.lengthX();
	}
}

uint worstBlockForStock(const std::vector<cg3::Dcel> &decomposition, const cg3::BoundingBox3& stock, double& factor){
	//looking for the worst for minEdge, maxEdge and Z
	uint worstMinEdge = 0, worstMaxEdge = 0, worstZ = 0;
	double maxMinEdge = 0, maxMaxEdge = 0, maxZ = 0;
	uint i = 0;
	for (const cg3::Dcel& block : decomposition){
		cg3::BoundingBox3 bb = block.boundingBox();
		double minEdge, maxEdge;
		double z = bb.lengthZ();
		minMaxEdge(bb, minEdge, maxEdge);
		if (minEdge > maxMaxEdge){
			maxMinEdge = minEdge;
			worstMinEdge = i;
		}
		if (maxEdge > maxMaxEdge){
			maxMaxEdge = maxEdge;
			worstMaxEdge = i;
		}
		if (z > maxZ){
			maxZ = z;
			worstZ = i;
		}
		i++;
	}

	double minStock, maxStock;

	minMaxEdge(stock, minStock, maxStock);

	//ratios will contain the maximum number of blocks that can be inserted in the stock in that direction
	double minRatio = minStock / maxMinEdge, maxRatio = maxStock / maxMaxEdge, zRatio = stock.lengthZ() / maxZ;

	if (minRatio <= maxRatio && minRatio <= zRatio){
		factor = minRatio;
		return worstMinEdge;
	}
	else if (maxRatio <= minRatio && maxRatio <= zRatio){
		factor = maxRatio;
		return worstMaxEdge;
	}
	else{
		factor = zRatio;
		return worstZ;
	}
}

std::vector< std::vector<std::pair<int, cg3::Point3d> > > packing(const std::vector<cg3::Dcel>& blocks, const cg3::BoundingBox3& stock, double distanceBetweenblocks)
{
	std::vector< std::vector<std::pair<int, cg3::Point3d> > > packs;
	std::set<uint> unpackedBlocks;
	uint lastUnpackedBlocksSize = 0;
	for (uint i = 0; i < blocks.size(); ++i) unpackedBlocks.insert(i);

	while(unpackedBlocks.size() > 0 && lastUnpackedBlocksSize != unpackedBlocks.size()){
		lastUnpackedBlocksSize = unpackedBlocks.size();

		std::vector<std::pair<int, cg3::Point3d> > actualPack;

		// Create some 'content' to work on.
		BinPack2D::ContentAccumulator<int> inputContent;

		for(uint i : unpackedBlocks) {

			// random size for this content
			int width  = blocks[i].boundingBox().lengthX() + distanceBetweenblocks;
			int height = blocks[i].boundingBox().lengthY() + distanceBetweenblocks;

			// whatever data you want to associate with this content

			int mycontent= i;

			// Add it
			inputContent += BinPack2D::Content<int>(mycontent, BinPack2D::Coord(), BinPack2D::Size(width, height), false);
		}
		// Sort the input content by size... usually packs better.
		inputContent.Sort();

		// Create some bins!
		BinPack2D::CanvasArray<int> canvasArray =
				BinPack2D::UniformCanvasArrayBuilder<int>(stock.lengthX(),stock.lengthY(),1).Build();

		// A place to store content that didnt fit into the canvas array.
		BinPack2D::ContentAccumulator<int> remainder;

		// try to pack content into the bins.
		canvasArray.Place( inputContent, remainder );

		// A place to store packed content.
		BinPack2D::ContentAccumulator<int> outputContent;

		// Read all placed content.
		canvasArray.CollectContent( outputContent );

		// parse output.
		typedef BinPack2D::Content<int>::Vector::iterator binpack2d_iterator;
		//printf("PLACED:\n");
		for( binpack2d_iterator itor = outputContent.Get().begin(); itor != outputContent.Get().end(); itor++ ) {

			const BinPack2D::Content<int> &content = *itor;

			// retreive your data.
			const int &myContent = content.content;

			unpackedBlocks.erase((uint)myContent);

			cg3::Point3d pos((double)content.coord.x, (double)content.coord.y, (double)content.coord.z);

			std::pair<int, cg3::Point3d> pair;
			pair.first = content.rotated ? -(myContent+1): (myContent+1);
			pair.second = pos;
			actualPack.push_back(pair);

			/*printf("\t%d of size %3dx%3d at position %3d,%3d,%2d rotated=%s\n\n",
				   myContent,
				   content.size.w,
				   content.size.h,
				   content.coord.x,
				   content.coord.y,
				   content.coord.z,
				   (content.rotated ? "yes":" no"));*/
		}
		if (actualPack.size() > 0)
			packs.push_back(actualPack);
	}
	//if (unpackedBlocks.size() > 0)
	//	std::cerr << "Some pieces cannot be putted on a pack with the given sizes\n";
	return packs;
}

}

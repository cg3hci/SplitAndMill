/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef USER_ACTION_H
#define USER_ACTION_H

#include <cg3/meshes/dcel/dcel.h>
#include "manipulable_boundingbox.h"

class UserAction
{
public:
	UserAction();

	UserAction(const cg3::Dcel& mesh, const Eigen::Matrix3d& rotMatrix);
	UserAction(const cg3::Dcel& mesh, const cg3::Dcel& block, const cg3::BoundingBox3& box, ManipulableBoundingBox::MillingDir millingDir);
	UserAction(const cg3::Dcel& mesh, uint nIters, double lambda, double mu, bool firstSmooth);

	typedef enum {SMOOTHING, ROTATE, CUT} ActionType;

	ActionType type() const;
	const cg3::Dcel& mesh() const;
	const cg3::Dcel& block() const;
	double lambda() const;
	double mu() const;
	uint nIterations() const;
	bool firstSmoothing() const;
	Eigen::Matrix3d rotationMatrix() const;
	cg3::BoundingBox3 box() const;
	ManipulableBoundingBox::MillingDir millingDir() const;

private:
	ActionType actionType;
	cg3::Dcel _mesh;
	cg3::Dcel _block;
	double _lambda, _mu;
	uint nIt;
	bool firstSmooth;
	Eigen::Matrix3d _rotation;
	cg3::BoundingBox3 _box;
	ManipulableBoundingBox::MillingDir boxDir;
};

#endif // USER_ACTION_H

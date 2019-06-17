/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef USER_ACTION_H
#define USER_ACTION_H

#include <cg3/meshes/dcel/dcel.h>
#include "data_structures/hf_box.h"

class UserAction
{
public:
	UserAction();

	UserAction(const cg3::Dcel& mesh, const Eigen::Matrix3d& rotMatrix, const Eigen::Matrix3d& actualRotationMatrix);
	UserAction(const cg3::Dcel& mesh, const HFBox& box);
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
	Eigen::Matrix3d actualRotationMatrix() const;
	HFBox box() const;

private:
	ActionType actionType;
	cg3::Dcel _mesh;
	double _lambda, _mu;
	uint nIt;
	bool firstSmooth;
	Eigen::Matrix3d _rotation;
	Eigen::Matrix3d actualRot;
	HFBox _box;
};

#endif // USER_ACTION_H

/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "user_action.h"

UserAction::UserAction()
{
}

UserAction::UserAction(const cg3::Dcel &mesh, const Eigen::Matrix3d &rotMatrix) :
	actionType(ROTATE),
	_mesh(mesh),
	_rotation(rotMatrix)
{
}

UserAction::UserAction(const cg3::Dcel &mesh, const cg3::Dcel &block, const cg3::BoundingBox3 &box, ManipulableBoundingBox::MillingDir millingDir) :
	actionType(CUT),
	_mesh(mesh),
	_block(block),
	_box(box),
	boxDir(millingDir)
{
}

UserAction::UserAction(const cg3::Dcel &mesh, uint nIters, double lambda, double mu, bool firstSmooth) :
	actionType(SMOOTHING),
	_mesh(mesh),
	_lambda(lambda),
	_mu(mu),
	nIt(nIters),
	firstSmooth(firstSmooth)
{
}

UserAction::ActionType UserAction::type() const
{
	return actionType;
}

const cg3::Dcel &UserAction::mesh() const
{
	return _mesh;
}

const cg3::Dcel &UserAction::block() const
{
	return _block;
}

double UserAction::lambda() const
{
	return _lambda;
}

double UserAction::mu() const
{
	return _mu;
}

uint UserAction::nIterations() const
{
	return nIt;
}

bool UserAction::firstSmoothing() const
{
	return firstSmooth;
}

Eigen::Matrix3d UserAction::rotationMatrix() const
{
	return _rotation;
}

cg3::BoundingBox3 UserAction::box() const
{
	return _box;
}

ManipulableBoundingBox::MillingDir UserAction::millingDir() const
{
	return boxDir;
}

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

UserAction::UserAction(const cg3::Dcel &mesh, const Eigen::Matrix3d &rotMatrix, const Eigen::Matrix3d &actualRotationMatrix, uint tab) :
	actionType(ROTATE),
	_mesh(mesh),
	_rotation(rotMatrix),
	actualRot(actualRotationMatrix),
	_tab(tab)
{
}

UserAction::UserAction(const cg3::Dcel &mesh, const HFBox &box, uint tab) :
	actionType(CUT),
	_mesh(mesh),
	_box(box),
	_tab(tab)
{
}

UserAction::UserAction(const cg3::Dcel &mesh, uint nIters, double lambda, double mu, bool firstSmooth, uint tab) :
	actionType(SMOOTHING),
	_mesh(mesh),
	_lambda(lambda),
	_mu(mu),
	nIt(nIters),
	firstSmooth(firstSmooth),
	_tab(tab)
{
}

UserAction::UserAction(const cg3::Dcel &mesh, const cg3::Dcel &restoredMesh, uint nIters, uint tab) :
	actionType(RESTORE_HIGH_FREQ),
	_mesh(mesh),
	_restoredMesh(restoredMesh),
	nIt(nIters),
	_tab(tab)
{
}

UserAction::UserAction(const std::vector<cg3::Dcel> &decomposition, uint tab) :
	actionType(DECOMPOSITION),
	_decomposition(decomposition),
	_tab(tab)
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

const cg3::Dcel &UserAction::restoredMesh() const
{
	return _restoredMesh;
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

Eigen::Matrix3d UserAction::actualRotationMatrix() const
{
	return actualRot;
}

HFBox UserAction::box() const
{
	return _box;
}

uint UserAction::tab() const
{
	return _tab;
}

const std::vector<cg3::Dcel> &UserAction::decomposition()
{
	return _decomposition;
}

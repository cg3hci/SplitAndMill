/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "hf_gui.h"
#include "ui_hf_gui.h"

#include <cg3/algorithms/global_optimal_rotation_matrix.h>
#include <cg3/algorithms/sphere_coverage.h>
#include <cg3/libigl/booleans.h>
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>

HFGui::HFGui(QWidget *parent) :
    QFrame(parent),
	ui(new Ui::HFGui),
	mw((cg3::viewer::MainWindow&)*parent),
	box(mw.canvas),
	actualAction(0)
{
    ui->setupUi(this);

	lsmesh.addSupportedExtension("obj");
	lsmesh.addSupportedExtension("ply");
	lsmesh.addSupportedExtension("dcel");
	mw.canvas.toggleCameraType();

	connect(&mw, SIGNAL(undoEvent()),
			this, SLOT(undo()));

	connect(&mw, SIGNAL(redoEvent()),
			this, SLOT(redo()));
}

HFGui::~HFGui()
{
	delete ui;
}

void HFGui::addAction(const UserAction &action)
{
	if(actualAction != actions.size()){
		actions.erase(actions.begin() + actualAction, actions.end());
	}
	actions.push_back(action);
	actualAction++;
}

void HFGui::undo()
{
	if (actualAction != 0){
		actualAction--;
		switch(actions[actualAction].type()){
		case UserAction::LOAD_MESH :
			mw.deleteDrawableObject(&mesh);
			mw.deleteDrawableObject(&box);
			mw.deleteDrawableObject(&hfDecomposition);
			hfDecomposition.clear();
			hfDirs.clear();
			break;
		case UserAction::SMOOTHING :
			break;
		case UserAction::ROTATE :
			break;
		case UserAction::CUT :
			bool b = mesh.isVisible();
			mesh = actions[actualAction].mesh();
			box.set(actions[actualAction].box().min(), actions[actualAction].box().max());
			box.setMillingDirection(actions[actualAction].millingDir());
			hfDecomposition.erase(hfDecomposition.size()-1);
			hfDirs.pop_back();
			mesh.update();
			mw.setDrawableObjectVisibility(&mesh, b);
			treeMesh = cg3::cgal::AABBTree3(mesh);
			break;
		}
		mw.canvas.update();
	}
}

void HFGui::redo()
{
	if (actualAction < actions.size()){
		switch(actions[actualAction].type()){
		case UserAction::LOAD_MESH :
			mesh = actions[actualAction].mesh();
			mw.pushDrawableObject(&mesh, "Loaded Mesh");
			mw.pushDrawableObject(&box, "Box");
			mw.pushDrawableObject(&hfDecomposition, "Decomposition", false);
			mesh.update();
			treeMesh = cg3::cgal::AABBTree3(mesh);
			break;
		case UserAction::SMOOTHING :
			break;
		case UserAction::ROTATE :
			break;
		case UserAction::CUT :
			bool v = mesh.isVisible();
			box.set(actions[actualAction].box().min(), actions[actualAction].box().max());
			cg3::SimpleEigenMesh b = cg3::EigenMeshAlgorithms::makeBox(box.min(), box.max());
			mesh = cg3::DrawableDcel(cg3::libigl::difference(mesh, b));
			box.setMillingDirection(actions[actualAction].millingDir());
			hfDecomposition.pushBack(actions[actualAction].block(), "");
			hfDirs.push_back(actions[actualAction].millingDir());
			mesh.update();
			treeMesh = cg3::cgal::AABBTree3(mesh);
			mw.setDrawableObjectVisibility(&mesh, v);
			break;
		}
		mw.canvas.update();
		actualAction++;
	}
}

void HFGui::on_loadMeshPushButton_clicked()
{
	std::string filename = lsmesh.loadDialog("Load Mesh");
	if (filename != ""){
		if(mesh.loadFromFile(filename)){
			mesh.translate(-mesh.boundingBox().center());
			mesh.setFaceColors(cg3::GREY);
			mesh.update();
			addAction(UserAction(mesh));
			mw.pushDrawableObject(&mesh, "Loaded Mesh");
			box.set(mesh.boundingBox().min(), mesh.boundingBox().max());
			mw.pushDrawableObject(&box, "box");
			mw.pushDrawableObject(&hfDecomposition, "HF Decomposition", false);
			treeMesh = cg3::cgal::AABBTree3(mesh);

			//mw.pushDrawableObject(&box.min(), "min");
			//mw.pushDrawableObject(&box.max(), "max");
			mw.canvas.fitScene();
			mw.canvas.update();
		}
	}
}

void HFGui::on_clearPushButton_clicked()
{
	mw.deleteDrawableObject(&mesh);
	mw.deleteDrawableObject(&box);
	mw.deleteDrawableObject(&hfDecomposition);
	hfDecomposition.clear();
	hfDirs.clear();
	mw.canvas.update();
}

void HFGui::on_pxRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(ManipulableBoundingBox::PLUS_X);
		mw.canvas.update();
	}
}

void HFGui::on_pyRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(ManipulableBoundingBox::PLUS_Y);
		mw.canvas.update();
	}
}

void HFGui::on_pzRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(ManipulableBoundingBox::PLUS_Z);
		mw.canvas.update();
	}
}

void HFGui::on_mxRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(ManipulableBoundingBox::MINUS_X);
		mw.canvas.update();
	}
}

void HFGui::on_myRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(ManipulableBoundingBox::MINUS_Y);
		mw.canvas.update();
	}
}

void HFGui::on_mzRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(ManipulableBoundingBox::MINUS_Z);
		mw.canvas.update();
	}
}

void HFGui::on_colorAllTrisPushButton_clicked()
{
	mesh.setFaceColors(cg3::GREY);
	cg3::Vec3 dir = cg3::AXIS[box.millingDirection()];
	for (cg3::Dcel::Face* f : mesh.faceIterator()) {
		if (f->normal().dot(dir) >= std::cos(98 * (M_PI / 180))){
			if (f->normal().dot(dir) >= -cg3::EPSILON)
				f->setColor(cg3::GREEN);
			else
				f->setColor(cg3::YELLOW);
		}
		else
			f->setColor(cg3::RED);
	}
	mesh.update();
	mw.canvas.update();
}

void HFGui::on_containedTrisPushButton_clicked()
{
	mesh.setFaceColors(cg3::GREY);
	cg3::Vec3 dir = cg3::AXIS[box.millingDirection()];
	std::list<const cg3::Dcel::Face*> lf = treeMesh.containedDcelFaces(cg3::BoundingBox3(box.min(), box.max()));
	for(const cg3::Dcel::Face* f : lf){
		if (f->normal().dot(dir) >= std::cos(98 * (M_PI / 180))){
			if (f->normal().dot(dir) >= -cg3::EPSILON)
				mesh.face(f->id())->setColor(cg3::GREEN);
			else
				mesh.face(f->id())->setColor(cg3::YELLOW);
		}
		else
			mesh.face(f->id())->setColor(cg3::RED);
	}
	mesh.update();
	mw.canvas.update();
}

void HFGui::on_optimalOrientationPushButton_clicked()
{
	uint nDirs = ui->nDirsSpinBox->value();
	std::vector<cg3::Vec3> dirs = cg3::sphereCoverageFibonacci(nDirs-6);
	dirs.insert(dirs.end(), cg3::AXIS.begin(), cg3::AXIS.end());
	Eigen::Matrix3d rot = cg3::globalOptimalRotationMatrix(mesh, dirs);
	addAction(UserAction(mesh, rot));
	mesh.rotate(rot);
	treeMesh = cg3::cgal::AABBTree3(mesh);
	mesh.update();
	mw.canvas.update();
}

void HFGui::on_cutPushButton_clicked()
{
	cg3::BoundingBox3 bb(box.min(), box.max());
	cg3::SimpleEigenMesh b = cg3::EigenMeshAlgorithms::makeBox(bb);
	cg3::DrawableDcel res = cg3::DrawableDcel(cg3::libigl::intersection(mesh, b));
	addAction(UserAction(mesh, res, bb, box.millingDirection()));
	mesh = cg3::DrawableDcel(cg3::libigl::difference(mesh, b));
	mesh.update();
	treeMesh = cg3::cgal::AABBTree3(mesh);
	hfDecomposition.pushBack(res, "");
	hfDirs.push_back(cg3::AXIS[box.millingDirection()]);
	mw.canvas.update();
}

void HFGui::on_manualRotationPushButton_clicked()
{
	cg3::Vec3 axis(ui->xAxisSpinBox->value(), ui->yAxisSpinBox->value(), ui->zAxisSpinBox->value());
	double angle = ui->angleSpinBox->value();
	Eigen::Matrix3d rot = cg3::rotationMatrix(axis, angle);
	addAction(UserAction(mesh, rot));
	mesh.rotate(rot);
	treeMesh = cg3::cgal::AABBTree3(mesh);
	mesh.update();
	mw.canvas.update();
}

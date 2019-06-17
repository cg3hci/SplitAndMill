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
#include <cg3/vcglib/smoothing.h>

HFGui::HFGui(QWidget *parent) :
    QFrame(parent),
	ui(new Ui::HFGui),
	mw((cg3::viewer::MainWindow&)*parent),
	rotatableMesh(mw.canvas),
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

void HFGui::clear()
{
	mw.deleteDrawableObject(&mesh);
	mw.deleteDrawableObject(&box);
	mw.deleteDrawableObject(&hfDecomposition);
	mw.deleteDrawableObject(&rotatableMesh);
	hfDecomposition.clear();
	hfDirs.clear();
	actions.clear();
	actualAction = 0;
	mw.canvas.update();
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
		case UserAction::SMOOTHING :
			mesh = actions[actualAction].mesh();
			if (actions[actualAction].firstSmoothing()){
				mw.deleteDrawableObject(&originalMesh);
				mw.setDrawableObjectName(&mesh, "Loaded Mesh");
			}
			break;
		case UserAction::ROTATE :
			mesh = actions[actualAction].mesh();
			break;
		case UserAction::CUT :
			bool b = mesh.isVisible();
			mesh = actions[actualAction].mesh();
			box.set(actions[actualAction].box().min(), actions[actualAction].box().max());
			box.setMillingDirection(actions[actualAction].millingDir());
			hfDecomposition.erase(hfDecomposition.size()-1);
			hfDirs.pop_back();
			mw.setDrawableObjectVisibility(&mesh, b);
			break;
		}
		mesh.update();
		treeMesh = cg3::cgal::AABBTree3(mesh);
		mw.canvas.update();
	}
}

void HFGui::redo()
{
	if (actualAction < actions.size()){
		Eigen::Matrix3d rot;
		switch(actions[actualAction].type()){
		case UserAction::SMOOTHING :
			if (actions[actualAction].firstSmoothing()){
				originalMesh = actions[actualAction].mesh();
				originalMesh.update();
				mw.pushDrawableObject(&originalMesh, "Non-Smoothed Mesh", false);
				mw.setDrawableObjectName(&mesh, "Smoothed Mesh");
			}
			mesh = (cg3::Dcel)cg3::vcglib::taubinSmoothing(mesh, actions[actualAction].nIterations(), actions[actualAction].lambda(), actions[actualAction].mu());
			break;
		case UserAction::ROTATE :
			rot = actions[actualAction].rotationMatrix();
			mesh.rotate(rot);
			break;
		case UserAction::CUT :
			bool v = mesh.isVisible();
			box.set(actions[actualAction].box().min(), actions[actualAction].box().max());
			cg3::SimpleEigenMesh b = cg3::EigenMeshAlgorithms::makeBox(box.min(), box.max());
			mesh = cg3::DrawableDcel(cg3::libigl::difference(mesh, b));
			box.setMillingDirection(actions[actualAction].millingDir());
			hfDecomposition.pushBack(actions[actualAction].block(), "");
			hfDirs.push_back(actions[actualAction].millingDir());
			mw.setDrawableObjectVisibility(&mesh, v);
			break;
		}
		mesh.update();
		treeMesh = cg3::cgal::AABBTree3(mesh);
		mw.canvas.update();
		actualAction++;
	}
}

void HFGui::on_loadMeshPushButton_clicked()
{
	std::string filename = lsmesh.loadDialog("Load Mesh");
	if (filename != ""){
		clear();
		if(mesh.loadFromFile(filename)){
			mesh.translate(-mesh.boundingBox().center());
			mesh.setFaceColors(cg3::GREY);
			mesh.update();
			mw.pushDrawableObject(&mesh, "Loaded Mesh");
			box.set(mesh.boundingBox().min(), mesh.boundingBox().max());
			mw.pushDrawableObject(&box, "box");
			mw.pushDrawableObject(&hfDecomposition, "HF Decomposition", false);
			treeMesh = cg3::cgal::AABBTree3(mesh);

			mw.canvas.fitScene();
			mw.canvas.update();
		}
	}
}

void HFGui::on_taubinSmoothingPushButton_clicked()
{
	bool firstSmooth = false;
	if (!mw.containsDrawableObject(&originalMesh)){
		originalMesh = mesh;
		originalMesh.update();
		mw.pushDrawableObject(&originalMesh, "Non-Smoothed Mesh", false);
		mw.setDrawableObjectName(&mesh, "Smoothed Mesh");
		firstSmooth = true;
	}
	addAction(UserAction(mesh, ui->nIterationsSpinBox->value(), ui->lambdaSpinBox->value(), ui->muSpinBox->value(), firstSmooth));
	mesh = (cg3::Dcel)cg3::vcglib::taubinSmoothing(mesh, ui->nIterationsSpinBox->value(), ui->lambdaSpinBox->value(), ui->muSpinBox->value());
	mesh.update();
	treeMesh = cg3::cgal::AABBTree3(mesh);

	mw.canvas.update();
}

void HFGui::on_clearPushButton_clicked()
{
	clear();
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
	hfDecomposition.pushBack(res, "Block " + std::to_string(hfDecomposition.size()));
	hfDirs.push_back(cg3::AXIS[box.millingDirection()]);
	mw.canvas.update();
}



void HFGui::on_manualOrientationCheckBox_stateChanged(int arg1)
{
	if (arg1 == Qt::Checked){
		mw.setDrawableObjectVisibility(&mesh, false);
		rotatableMesh.setMesh(mesh);
		mw.pushDrawableObject(&rotatableMesh, "Rot");
	}
	else {
		Eigen::Matrix3d rot = rotatableMesh.rotationMatrix();
		addAction(UserAction(mesh, rot));
		mesh.rotate(rot);
		mesh.update();
		treeMesh = cg3::cgal::AABBTree3(mesh);
		mw.deleteDrawableObject(&rotatableMesh);
		mw.setDrawableObjectVisibility(&mesh, true);
	}
	mw.canvas.update();
}

void HFGui::on_saveDecompositionPushButton_clicked()
{
	std::string dir = lsmesh.directoryDialog();
	if (dir != "") {
		uint i = 0;
		for (const cg3::DrawableDcel& b : hfDecomposition){
			b.saveOnObj(dir + "b" + std::to_string(i) + ".obj", false);
		}
	}
}

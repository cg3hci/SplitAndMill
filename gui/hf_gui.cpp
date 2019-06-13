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
	mw((cg3::viewer::MainWindow&)*parent)
{
    ui->setupUi(this);

	lsmesh.addSupportedExtension("obj");
	lsmesh.addSupportedExtension("ply");
	lsmesh.addSupportedExtension("dcel");
	mw.canvas.toggleCameraType();
}

HFGui::~HFGui()
{
    delete ui;
}

void HFGui::on_loadMeshPushButton_clicked()
{
	std::string filename = lsmesh.loadDialog("Load Mesh");
	if (filename != ""){
		if(mesh.loadFromFile(filename)){
			mesh.translate(-mesh.boundingBox().center());
			mesh.update();
			mw.pushDrawableObject(&mesh, "Loaded Mesh");
			box.set(mesh.boundingBox().min(), mesh.boundingBox().max());
			mw.pushDrawableObject(&box, "box");
			mw.pushDrawableObject(&hfDecomposition, "HF Decomposition", false);
			treeMesh = cg3::cgal::AABBTree(mesh);

			//mw.pushDrawableObject(&box.min(), "min");
			//mw.pushDrawableObject(&box.max(), "max");
			mw.canvas.fitScene();
			mw.canvas.update();
		}
	}
}

void HFGui::on_resetBoxPushButton_clicked()
{
	qglviewer::Quaternion q;
	box.setRotation(q);
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

void HFGui::on_containedTrisPushButton_clicked()
{
	mesh.setColor(cg3::GREY);
	cg3::Vec3 dir = cg3::AXIS[box.millingDirection()];
	std::list<const cg3::Dcel::Face*> lf = treeMesh.containedDcelFaces(cg3::BoundingBox(box.min(), box.max()));
	for(const cg3::Dcel::Face* f : lf){
		if (f->normal().dot(dir) >= -cg3::EPSILON)
			mesh.face(f->id())->setColor(cg3::GREEN);
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
	mesh.rotate(rot);
	//box.set(mesh.boundingBox().min(), mesh.boundingBox().max());
	treeMesh = cg3::cgal::AABBTree(mesh);
	mesh.update();
	mw.canvas.update();
}

void HFGui::on_cutPushButton_clicked()
{
	cg3::SimpleEigenMesh b = cg3::EigenMeshAlgorithms::makeBox(box.min(), box.max());
	cg3::DrawableDcel res = cg3::DrawableDcel(cg3::libigl::intersection(mesh, b));
	mesh = cg3::DrawableDcel(cg3::libigl::difference(mesh, b));
	mesh.update();
	treeMesh = cg3::cgal::AABBTree(mesh);
	hfDecomposition.pushBack(res, "");
	hfDirs.push_back(cg3::AXIS[box.millingDirection()]);
	mw.canvas.update();
}

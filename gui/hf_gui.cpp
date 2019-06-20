/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "hf_gui.h"
#include "ui_hf_gui.h"

#include <QMessageBox>

#include <cg3/algorithms/global_optimal_rotation_matrix.h>
#include <cg3/algorithms/sphere_coverage.h>
#include <cg3/libigl/booleans.h>
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>
#include <cg3/vcglib/smoothing.h>

HFGui::HFGui(QWidget *parent) :
    QFrame(parent),
	ui(new Ui::HFGui),
	mw((cg3::viewer::MainWindow&)*parent),
	actualRotationMatrix(Eigen::Matrix3d::Identity()),
	actualTab(0),
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
	hfEngine.clear();
	actions.clear();
	actualAction = 0;
	mw.canvas.update();

	ui->tabWidget->setCurrentIndex(0);
	ui->tabWidget->setEnabled(false);
	ui->testFrame->setEnabled(false);
}

void HFGui::addAction(const UserAction &action)
{
	if(actualAction != actions.size()){
		actions.erase(actions.begin() + actualAction, actions.end());
	}
	actions.push_back(action);
	actualAction++;
}

void HFGui::updateSurfaceAndvolume()
{
	remainingSurface = 0;
	for (cg3::Dcel::Face* f : mesh.faceIterator()){
		if (f->flag() != 1) {
			remainingSurface+=f->area();
		}
	}

	remainingVolume = mesh.volume();
	double percV = remainingVolume / totalVolume * 100;
	double percS = remainingSurface / totalSurface * 100;

	ui->remainingVolumeLabel->setText(QString::fromStdString(std::to_string(percV)) + " %");
	ui->remainingSurfaceLabel->setText(QString::fromStdString(std::to_string(percS)) + " %");
}

void HFGui::changeTab(uint tab)
{
	ui->tabWidget->setTabEnabled(actualTab, false);
	ui->tabWidget->setTabEnabled(tab, true);
	ui->tabWidget->setCurrentIndex(tab);
	actualTab = tab;
	if (tab >= 2){
		ui->testOrTrianglesCheckBox->setChecked(false);
		ui->testFrame->setEnabled(false);
	}
	else {
		ui->testFrame->setEnabled(true);
	}
}

int HFGui::selectedTestdirection() const
{
	if (ui->testOrTrianglesCheckBox->isChecked()){
		if (ui->pxTestRadioButton->isChecked())
			return 0;
		if (ui->pyTestRadioButton->isChecked())
			return 1;
		if (ui->pzTestRadioButton->isChecked())
			return 2;
		if (ui->mxTestRadioButton->isChecked())
			return 3;
		if (ui->myTestRadioButton->isChecked())
			return 4;
		if (ui->mzTestRadioButton->isChecked())
			return 5;
	}
	else {
		return -1;
	}
	return -1;
}

void HFGui::colorTestMesh()
{
	int d = selectedTestdirection();
	if (d < 0){
		mesh.setFaceColors(cg3::GREY);

	}
	else {
		cg3::Vec3 dir = cg3::AXIS [d];
		for (cg3::Dcel::Face* f : mesh.faceIterator()) {
			if (f->normal().dot(dir) >= std::cos(ui->flipAngleSpinBox->value() * (M_PI / 180))){
				if (f->normal().dot(dir) >= std::cos(ui->lightToleranceSpinBox->value() * (M_PI / 180)))
					f->setColor(cg3::GREEN);
				else
					f->setColor(cg3::YELLOW);
			}
			else
				f->setColor(cg3::RED);
		}
	}
	mesh.update();
	mw.canvas.update();
}

void HFGui::undo()
{
	if (actualAction != 0){
		actualAction--;
		switch(actions[actualAction].type()){
		case UserAction::SMOOTHING :
			mesh = actions[actualAction].mesh();
			hfEngine.setMesh(mesh);
			if (actions[actualAction].firstSmoothing()){
				hfEngine.setUseSmoothedMesh(false);
				mw.deleteDrawableObject(&originalMesh);
				mw.setDrawableObjectName(&mesh, "Loaded Mesh");
			}
			totalVolume = mesh.volume();
			totalSurface = mesh.surfaceArea();
			remainingVolume = totalVolume;
			remainingSurface = totalSurface;
			changeTab(actions[actualAction].tab());
			break;
		case UserAction::ROTATE :
			mesh = actions[actualAction].mesh();
			actualRotationMatrix = actions[actualAction].actualRotationMatrix(); //avoid numerical errors
			changeTab(actions[actualAction].tab());
			break;
		case UserAction::CUT :
			bool b = mesh.isVisible();
			mesh = actions[actualAction].mesh();
			box.set(actions[actualAction].box().min(), actions[actualAction].box().max());
			box.setMillingDirection(actions[actualAction].box().millingDirection());
			hfEngine.popBox();
			mw.setDrawableObjectVisibility(&mesh, b);
			updateSurfaceAndvolume();
			changeTab(actions[actualAction].tab());
			if (remainingVolume == totalVolume){
				ui->flipAngleSpinBox->setEnabled(true);
				ui->lightToleranceSpinBox->setEnabled(true);
			}
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
				hfEngine.setOriginalMesh(originalMesh);
				originalMesh.update();
				mw.pushDrawableObject(&originalMesh, "Non-Smoothed Mesh", false);
				mw.setDrawableObjectName(&mesh, "Smoothed Mesh");
			}
			mesh = (cg3::Dcel)cg3::vcglib::taubinSmoothing(mesh, actions[actualAction].nIterations(), actions[actualAction].lambda(), actions[actualAction].mu());
			hfEngine.setMesh(mesh);
			totalVolume = mesh.volume();
			totalSurface = mesh.surfaceArea();
			remainingVolume = totalVolume;
			remainingSurface = totalSurface;
			changeTab(0);
			colorTestMesh();
			break;
		case UserAction::ROTATE :
			rot = actions[actualAction].rotationMatrix();
			actualRotationMatrix *= rot;
			mesh.rotate(rot);
			changeTab(1);
			colorTestMesh();
			break;
		case UserAction::CUT :
			bool v = mesh.isVisible();
			box.set(actions[actualAction].box().min(), actions[actualAction].box().max());
			cg3::SimpleEigenMesh b = cg3::EigenMeshAlgorithms::makeBox(box.min(), box.max());
			mesh = cg3::DrawableDcel(cg3::libigl::difference(mesh, b));
			box.setMillingDirection(actions[actualAction].box().millingDirection());
			hfEngine.pushBox(actions[actualAction].box());
			mw.setDrawableObjectVisibility(&mesh, v);
			updateSurfaceAndvolume();
			changeTab(2);
			ui->flipAngleSpinBox->setEnabled(false);
			ui->lightToleranceSpinBox->setEnabled(false);
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

			mesh.setFaceFlags(0);

			mw.pushDrawableObject(&mesh, "Loaded Mesh");
			treeMesh = cg3::cgal::AABBTree3(mesh);
			hfEngine.setMesh(mesh);

			mw.canvas.fitScene();
			mw.canvas.update();

			ui->tabWidget->setEnabled(true);
			ui->tabWidget->setTabEnabled(0, true);
			ui->tabWidget->setTabEnabled(1, false);
			ui->tabWidget->setTabEnabled(2, false);
			ui->tabWidget->setTabEnabled(3, false);
			ui->tabWidget->setTabEnabled(4, false);
			ui->testFrame->setEnabled(true);
			actualTab = 0;
		}
	}
}

void HFGui::on_clearPushButton_clicked()
{
	clear();
}

void HFGui::on_exportDecompositionPushButton_clicked()
{
	std::string dir = lsmesh.directoryDialog();
	if (dir != "") {
		uint i = 0;
		for (const cg3::DrawableDcel& b : hfDecomposition){
			b.saveOnObj(dir + "b" + std::to_string(i) + ".obj", false);
		}
	}
}

void HFGui::on_taubinSmoothingPushButton_clicked()
{
	bool firstSmooth = false;
	mesh.setFaceColors(cg3::GREY);
	if (!mw.containsDrawableObject(&originalMesh)){
		originalMesh = mesh;
		originalMesh.update();
		hfEngine.setOriginalMesh(originalMesh);
		mw.pushDrawableObject(&originalMesh, "Non-Smoothed Mesh", false);
		mw.setDrawableObjectName(&mesh, "Smoothed Mesh");
		firstSmooth = true;
	}
	addAction(UserAction(mesh, ui->nIterationsSpinBox->value(), ui->lambdaSpinBox->value(), ui->muSpinBox->value(), firstSmooth, actualTab));
	mesh = (cg3::Dcel)cg3::vcglib::taubinSmoothing(mesh, ui->nIterationsSpinBox->value(), ui->lambdaSpinBox->value(), ui->muSpinBox->value());
	mesh.setFaceFlags(0);
	mesh.setFaceColors(cg3::GREY);
	hfEngine.setMesh(mesh);
	colorTestMesh();
	treeMesh = cg3::cgal::AABBTree3(mesh);

	mw.canvas.update();
}

void HFGui::on_smoothingNextPushButton_clicked()
{
	box.set(mesh.boundingBox().min(), mesh.boundingBox().max());
	mw.pushDrawableObject(&box, "box");
	changeTab(1);
	totalVolume = mesh.volume();
	totalSurface = mesh.surfaceArea();
	remainingVolume = totalVolume;
	remainingSurface = totalSurface;
	mw.canvas.update();
}

void HFGui::on_automaticOrientationRadioButton_toggled(bool checked)
{
	if (checked){
		ui->optimalOrientationPushButton->setEnabled(true);
		ui->nDirsSpinBox->setEnabled(true);
		ui->resetRotationPushButton->setEnabled(false);
		ui->manualOrientationDonePushButton->setEnabled(false);

		mw.setDrawableObjectVisibility(&mesh, true);
		mw.deleteDrawableObject(&rotatableMesh);
		mw.canvas.update();
	}
}

void HFGui::on_manualOrientationRadioButton_toggled(bool checked)
{
	if (checked){
		ui->optimalOrientationPushButton->setEnabled(false);
		ui->nDirsSpinBox->setEnabled(false);
		ui->resetRotationPushButton->setEnabled(true);
		ui->manualOrientationDonePushButton->setEnabled(true);

		mw.setDrawableObjectVisibility(&mesh, false);
		rotatableMesh.setMesh(mesh);
		mw.pushDrawableObject(&rotatableMesh, "Rot");
		mw.canvas.update();
	}
}

void HFGui::on_optimalOrientationPushButton_clicked()
{
	uint nDirs = ui->nDirsSpinBox->value();
	std::vector<cg3::Vec3> dirs = cg3::sphereCoverageFibonacci(nDirs-6);
	dirs.insert(dirs.end(), cg3::AXIS.begin(), cg3::AXIS.end());
	Eigen::Matrix3d rot = cg3::globalOptimalRotationMatrix(mesh, dirs);
	addAction(UserAction(mesh, rot, actualRotationMatrix, actualTab));
	actualRotationMatrix *= rot;
	mesh.rotate(rot);
	colorTestMesh();
	treeMesh = cg3::cgal::AABBTree3(mesh);
	mw.canvas.update();
}

void HFGui::on_resetRotationPushButton_clicked()
{
	rotatableMesh.resetRotation();
	mw.canvas.update();
}

void HFGui::on_manualOrientationDonePushButton_clicked()
{
	Eigen::Matrix3d rot = rotatableMesh.rotationMatrix();
	addAction(UserAction(mesh, rot, actualRotationMatrix, actualTab));
	actualRotationMatrix *= rot;
	mesh.rotate(rot);
	colorTestMesh();
	treeMesh = cg3::cgal::AABBTree3(mesh);
	mw.canvas.update();
	ui->automaticOrientationRadioButton->toggle();
}

void HFGui::on_orientationNextPushButton_clicked()
{
	changeTab(2);
	colorTestMesh();

	box.setDrawArrow(true);
	mw.canvas.update();
}

void HFGui::on_pxRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(HFBox::PLUS_X);
		mw.canvas.update();
	}
}

void HFGui::on_pyRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(HFBox::PLUS_Y);
		mw.canvas.update();
	}
}

void HFGui::on_pzRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(HFBox::PLUS_Z);
		mw.canvas.update();
	}
}

void HFGui::on_mxRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(HFBox::MINUS_X);
		mw.canvas.update();
	}
}

void HFGui::on_myRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(HFBox::MINUS_Y);
		mw.canvas.update();
	}
}

void HFGui::on_mzRadioButton_toggled(bool checked)
{
	if (checked){
		box.setMillingDirection(HFBox::MINUS_Z);
		mw.canvas.update();
	}
}

void HFGui::on_colorAllTrisPushButton_clicked()
{
	cg3::Vec3 dir = cg3::AXIS[box.millingDirection()];
	for (cg3::Dcel::Face* f : mesh.faceIterator()) {
		if (f->normal().dot(dir) >= std::cos(ui->flipAngleSpinBox->value() * (M_PI / 180))){
			if (f->normal().dot(dir) >= std::cos(ui->lightToleranceSpinBox->value() * (M_PI / 180)))
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
		if (f->normal().dot(dir) >= std::cos(ui->flipAngleSpinBox->value() * (M_PI / 180))){
			if (f->normal().dot(dir) >= std::cos(ui->lightToleranceSpinBox->value() * (M_PI / 180)))
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

void HFGui::on_cutPushButton_clicked()
{
	ui->flipAngleSpinBox->setEnabled(false);
	ui->lightToleranceSpinBox->setEnabled(false);

	cg3::BoundingBox3 bb(box.min(), box.max());
	cg3::SimpleEigenMesh b = cg3::EigenMeshAlgorithms::makeBox(bb);
	HFBox hfbox(box.min(), box.max(), box.millingDirection(), actualRotationMatrix);
	hfEngine.pushBox(hfbox);
	addAction(UserAction(mesh, hfbox, actualTab));

	std::vector<uint> birthFaces;
	uint nFaces = mesh.numberFaces();

	cg3::Dcel res = cg3::DrawableDcel(cg3::libigl::difference(mesh, b, birthFaces));
	for (cg3::Dcel::Face* f : res.faceIterator()){
		if (birthFaces[f->id()] < nFaces){
			f->setFlag(mesh.face(birthFaces[f->id()])->flag());
//			if (f->flag() == 1)
//				f->setColor(cg3::RED);
		}
		else{
			f->setFlag(1);
//			f->setColor(cg3::RED);
		}
	}

	mesh = res;
	mesh.update();
	treeMesh = cg3::cgal::AABBTree3(mesh);

	updateSurfaceAndvolume();

	mw.canvas.update();
}

void HFGui::on_decompositionNextPushButton_clicked()
{
	if (remainingSurface > 0){
		QMessageBox* box = new QMessageBox(&mw);
		box->setWindowTitle("Remaining Surface");
		box->setText("There is some remaining surface of the input model in the mesh.\n"
					 "What do you want to do with that?");
		box->addButton(QMessageBox::Ok);
		box->button(QMessageBox::Ok)->setText("Ignore Remaining Surface");
		box->addButton(QMessageBox::Cancel);
		box->button(QMessageBox::Cancel)->setText("Continue Decomposition");
		box->setEscapeButton(QMessageBox::Cancel);
		int ret = box->exec();
		if (ret == QMessageBox::Ok)
			finishDecomposition();
	}
	else
		finishDecomposition();
}

void HFGui::finishDecomposition()
{
	changeTab(3);

	if (mw.containsDrawableObject(&originalMesh)){
		ui->restoreHighFrequenciesPushButton->setEnabled(true);
		ui->nRestoreIterationsSpinBox->setEnabled(true);
		ui->nRestoreIterationsLabel->setEnabled(true);
		ui->hausDescrLabel->setEnabled(true);
		ui->hausdorffDistanceLabel->setEnabled(true);
		originalMesh = hfEngine.originalMesh();
		mw.refreshDrawableObject(&originalMesh);
	}

	mw.deleteDrawableObject(&box);
	mesh = hfEngine.mesh();
	mw.refreshDrawableObject(&mesh);

	mw.canvas.update();
}

void HFGui::on_restoreHighFrequenciesPushButton_clicked()
{
	hfEngine.restoreHighFrequencies(ui->nIterationsSpinBox->value(), std::cos(ui->flipAngleSpinBox->value() * (M_PI / 180)));
	mesh = hfEngine.mesh();
	mw.refreshDrawableObject(&mesh);

	mw.canvas.update();
}

void HFGui::on_computeDecompositionPushButton_clicked()
{
	std::vector<cg3::Dcel> dec = hfEngine.decomposition();
	uint i = 0;
	for (const cg3::Dcel& d : dec)
		hfDecomposition.pushBack(d, "Block " + std::to_string(i));

	mw.setDrawableObjectVisibility(&mesh, false);
	mw.setDrawableObjectVisibility(&originalMesh, false);
	mw.pushDrawableObject(&hfDecomposition, "Decomposition");
	mw.canvas.update();
	ui->exportDecompositionPushButton->setEnabled(true);
	ui->nextPostProcessingPushButton->setEnabled(true);
}

void HFGui::on_testOrTrianglesCheckBox_stateChanged(int arg1)
{
	bool b = arg1 == Qt::Checked;
	ui->pxTestRadioButton->setEnabled(b);
	ui->pyTestRadioButton->setEnabled(b);
	ui->pzTestRadioButton->setEnabled(b);
	ui->mxTestRadioButton->setEnabled(b);
	ui->myTestRadioButton->setEnabled(b);
	ui->mzTestRadioButton->setEnabled(b);
	colorTestMesh();
}

void HFGui::on_pxTestRadioButton_toggled(bool checked)
{
	if (checked)
		colorTestMesh();
}

void HFGui::on_pyTestRadioButton_toggled(bool checked)
{
	if (checked)
		colorTestMesh();
}

void HFGui::on_pzTestRadioButton_toggled(bool checked)
{
	if (checked)
		colorTestMesh();
}

void HFGui::on_mxTestRadioButton_toggled(bool checked)
{
	if (checked)
		colorTestMesh();
}

void HFGui::on_myTestRadioButton_toggled(bool checked)
{
	if (checked)
		colorTestMesh();
}

void HFGui::on_mzTestRadioButton_toggled(bool checked)
{
	if (checked)
		colorTestMesh();
}

/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "hf_gui.h"
#include "ui_hf_gui.h"

HFGui::HFGui(QWidget *parent) :
    QFrame(parent),
	ui(new Ui::HFGui),
	mw((cg3::viewer::MainWindow&)*parent)
{
    ui->setupUi(this);

	lsmesh.addSupportedExtension("obj");
	lsmesh.addSupportedExtension("ply");
	lsmesh.addSupportedExtension("dcel");
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
			mw.pushDrawableObject(&mesh, "Loaded Mesh");
			box.set(mesh.boundingBox().min(), mesh.boundingBox().max());
			mw.pushDrawableObject(&box, "box");

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

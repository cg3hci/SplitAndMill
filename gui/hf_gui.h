/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef CG3_EXAMPLE_MANAGER_H
#define CG3_EXAMPLE_MANAGER_H

#include <QFrame>
#include <cg3/viewer/mainwindow.h>

#include <cg3/viewer/drawable_objects/drawable_dcel.h>
#include <cg3/viewer/utilities/loadersaver.h>

#include "manipulableboundingbox.h"

namespace Ui {
class HFGui;
}

class HFGui : public QFrame
{
    Q_OBJECT

public:
	explicit HFGui(QWidget *parent = 0);
	~HFGui();

private slots:
	void on_loadMeshPushButton_clicked();

	void on_resetBoxPushButton_clicked();

	void on_pxRadioButton_toggled(bool checked);

	void on_pyRadioButton_toggled(bool checked);

	void on_pzRadioButton_toggled(bool checked);

	void on_mxRadioButton_toggled(bool checked);

	void on_myRadioButton_toggled(bool checked);

	void on_mzRadioButton_toggled(bool checked);

private:
	Ui::HFGui *ui;

    //reference to the MainWindow
    cg3::viewer::MainWindow& mw;
	cg3::viewer::LoaderSaver lsmesh;

	cg3::DrawableDcel mesh;
	ManipulableBoundingBox box;
};

#endif // CG3_EXAMPLE_MANAGER_H

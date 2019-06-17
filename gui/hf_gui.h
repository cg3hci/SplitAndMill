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
#include <cg3/viewer/drawable_objects/drawable_objects_container.h>
#include <cg3/viewer/utilities/loadersaver.h>
#include <cg3/cgal/aabb_tree3.h>

#include "manipulable_boundingbox.h"
#include "rotatable_mesh.h"
#include "data_structures/user_action.h"
#include "data_structures/hf_box.h"

namespace Ui {
class HFGui;
}

class HFGui : public QFrame
{
    Q_OBJECT

public:
	explicit HFGui(QWidget *parent = 0);
	~HFGui();

	void clear();
	void addAction(const UserAction& action);

public slots:
	void undo();
	void redo();

private slots:

	//load/save
	void on_loadMeshPushButton_clicked();

	void on_clearPushButton_clicked();

	void on_exportDecompositionPushButton_clicked();

	//smoothing
	void on_taubinSmoothingPushButton_clicked();

	void on_smoothingNextPushButton_clicked();

	//orientation
	void on_automaticOrientationRadioButton_toggled(bool checked);

	void on_manualOrientationRadioButton_toggled(bool checked);

	void on_optimalOrientationPushButton_clicked();

	void on_resetRotationPushButton_clicked();

	void on_manualOrientationDonePushButton_clicked();

	void on_orientationNextPushButton_clicked();

	//box
	void on_pxRadioButton_toggled(bool checked);

	void on_pyRadioButton_toggled(bool checked);

	void on_pzRadioButton_toggled(bool checked);

	void on_mxRadioButton_toggled(bool checked);

	void on_myRadioButton_toggled(bool checked);

	void on_mzRadioButton_toggled(bool checked);

	void on_colorAllTrisPushButton_clicked();

	void on_containedTrisPushButton_clicked();

	void on_cutPushButton_clicked();

private:
	Ui::HFGui *ui;

    //reference to the MainWindow
    cg3::viewer::MainWindow& mw;
	cg3::viewer::LoaderSaver lsmesh;

	cg3::DrawableDcel mesh;
	cg3::DrawableDcel originalMesh;
	cg3::cgal::AABBTree3 treeMesh;
	Eigen::Matrix3d actualRotationMatrix;

	std::vector<HFBox> hfBoxes;
	cg3::DrawableObjectsContainer<cg3::DrawableDcel> hfDecomposition;


	RotatableMesh rotatableMesh;
	ManipulableBoundingBox box;

	std::vector<UserAction> actions;
	uint actualAction;
};

#endif // CG3_EXAMPLE_MANAGER_H

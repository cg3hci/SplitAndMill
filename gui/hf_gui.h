/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef CG3_EXAMPLE_MANAGER_H
#define CG3_EXAMPLE_MANAGER_H

#include <QFrame>
#include <QThread>
#include "hfmainwindow.h"

#include <cg3/viewer/drawable_objects/drawable_dcel.h>
#include <cg3/viewer/drawable_objects/drawable_objects_container.h>
#include <cg3/viewer/utilities/loadersaver.h>
#include <cg3/cgal/aabb_tree3.h>

#include "manipulable_boundingbox.h"
#include "rotatable_mesh.h"
#include "guides.h"
#include "data_structures/user_action.h"
#include "data_structures/hf_box.h"
#include "gui/hf_engine_thread.h"

namespace Ui {
class HFGui;
}

class HFGui : public QFrame
{
    Q_OBJECT

public:
	explicit HFGui(QWidget *parent = 0);
	~HFGui();

	bool loadMesh();
	bool loadHFD(std::string &filename);
	void afterLoadHFD();
	bool saveHFDAs(std::string &filename);
	bool saveHFD(const std::string& filename);
	bool decompositionComputed();
	bool saveDecomposition();
	bool packingComputed();
	bool savePacking();
	void drawBox();
	bool boxIsDrawn();

private:
	void clear();
	void addAction(const UserAction& action);
	void updateSurfaceAndvolume();
	void changeTab(uint tab);
	void startWork();
	void endWork();
	int selectedTestdirection() const;
	void colorTestMesh();

	void undoChangeTab();
	void redoChangeTab();
	void undoSmoothing();
	void redoSmoothing();
	void undoRotate();
	void redoRotate();
	void undoCut();
	void redoCut();
	void undoRestore();
	void redoRestore();
	void undoDecomposition();
	void redoDecomposition();

signals:
	void taubinSmoothing(uint, double, double);
	void optimalOrientation(uint);
	void restoreHighFrequencies(uint, double);
	void computeDecomposition();
	void computeDecompositionExact();
	void cut(cg3::Dcel, HFBox);
	void computeOneStockPackingFromDecomposition(cg3::BoundingBox3, double, double, cg3::Point2d, double);

private slots:

	void undo();
	void redo();
	void setProgressBarValue(uint value);
	void setUpSignals();

	//smoothing
	void on_taubinSmoothingPushButton_clicked();
	void taubinSmoothingCompleted();

	//orientation
	void on_automaticOrientationRadioButton_toggled(bool checked);

	void on_manualOrientationRadioButton_toggled(bool checked);

	void on_optimalOrientationPushButton_clicked();
	void optimalOrientationCompleted(Eigen::Matrix3d rot);

	void on_resetRotationPushButton_clicked();

	void on_manualOrientationDonePushButton_clicked();

	//test frame
	void on_testOrTrianglesCheckBox_stateChanged(int arg1);

	void on_pxTestRadioButton_toggled(bool checked);

	void on_pyTestRadioButton_toggled(bool checked);

	void on_pzTestRadioButton_toggled(bool checked);

	void on_mxTestRadioButton_toggled(bool checked);

	void on_myTestRadioButton_toggled(bool checked);

	void on_mzTestRadioButton_toggled(bool checked);

	void on_preProcessingNextPushButton_clicked();

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
	void startCut();
	void cutCompleted(cg3::Dcel res);

	void on_xGuidesCheckBox_stateChanged(int arg1);

	void on_yGuidesCheckBox_stateChanged(int arg1);

	void on_zGuidesCheckBox_stateChanged(int arg1);

	void on_snapMinXPushButton_clicked();

	void on_snapMaxXPushButton_clicked();

	void on_snapMinYPushButton_clicked();

	void on_snapMaxYPushButton_clicked();

	void on_snapMinZPushButton_clicked();

	void on_snapMaxZPushButton_clicked();

	void on_decompositionNextPushButton_clicked();
	void finishDecomposition();

	//post processing

	void on_restoreHighFrequenciesPushButton_clicked();
	void restoreHighFrequenciesCompleted();

	void on_computeDecompositionPushButton_clicked();
	void computeDecompositionCompleted();

	void on_nextPostProcessingPushButton_clicked();

	//packing

	void on_xStockSpinBox_valueChanged(int arg1);

	void on_yStockSpinBox_valueChanged(int arg1);

	void on_zStockSpinBox_valueChanged(int arg1);

	void on_clearPackingPushButton_clicked();

	void on_packPushButton_clicked();

	void on_packOneStockButton_clicked();
	void computeOneStockPackingFromDecompositionCompleted(bool success);



private:
	Ui::HFGui *ui;
	static const uint NTABS = 4;
	QFrame* tabs[NTABS];
	QLabel* tabLabels[NTABS];
	QThread workerThread;

    //reference to the MainWindow
	HFMainWindow& mw;
	cg3::viewer::LoaderSaver lsmesh;
	cg3::viewer::LoaderSaver lshfd;

	cg3::DrawableDcel mesh;
	cg3::DrawableDcel originalMesh;
	cg3::cgal::AABBTree3 treeMesh;
	Eigen::Matrix3d actualRotationMatrix;
	double totalSurface, totalVolume;
	double remainingSurface, remainingVolume;
	cg3::DrawableObjectsContainer<cg3::DrawableDcel> hfDecomposition;
	cg3::DrawableObjectsContainer<cg3::DrawableObjectsContainer<cg3::DrawableDcel>> packing;
	cg3::DrawableBoundingBox3 stock;
	uint actualTab;

	HFEngineThread* hfEngine;

	RotatableMesh rotatableMesh;
	ManipulableBoundingBox box;
	Guides guides;
	cg3::DrawableBoundingBox3 drawableBox;

	std::vector<UserAction> actions;
	uint actualAction;

	std::vector<cg3::Dcel> tmpPacking;
	const uint version = 2;

};

#endif // CG3_EXAMPLE_MANAGER_H

/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @author Marco Livesu (marco.livesu@gmail.com)
 */

#ifndef HFMAINWINDOW_H
#define HFMAINWINDOW_H

#include <QMainWindow>
#include <QApplication>
#include <QSignalMapper>
#include <QFrame>

#include <QProcess>
#ifdef __APPLE__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wredeclared-class-member"
#endif
#include <boost/bimap.hpp>
#ifdef __APPLE__
#pragma clang diagnostic pop
#endif

#include <cg3/viewer/interfaces/abstract_mainwindow.h>
#include <cg3/viewer/glcanvas.h>
#include <cg3/viewer/utilities/loadersaver.h>
#include <cg3/viewer/drawable_objects/drawable_mixed_objects.h>

#ifdef CG3_DCEL_DEFINED
#include <cg3/viewer/drawable_objects/drawable_dcel.h>
#include <cg3/viewer/drawable_objects/drawable_objects_container.h>
#endif

class QToolBox;
class QVBoxLayout;
class QSpacerItem;

class HFGui;

namespace cg3 {

class DrawableObject;
class PickableObject;
class DrawableContainer;

namespace viewer {

class GLCanvas;
class DrawableObjectDrawListManager;

}
}

namespace Ui {

class HFMainWindow;

} //namespace cg3::viewer::Ui

namespace internal {

//class MeshManager;
class UiMainWindowRaiiWrapper;

} //namespace cg3::viewer::internal

/**
 * @brief Class that describes a Window containing an QGLViewer canvas and that manages
 * Managers and DrawableObjects.
 * @ingroup cg3viewer
 */
class HFMainWindow : public cg3::viewer::AbstractMainWindow
{
    Q_OBJECT

public:

	explicit HFMainWindow(QWidget *parent = 0);
	~HFMainWindow();

	void setWidget(HFGui* frame);
	void setLoadedButtons(bool b);
	void setSaveDecompositionButtons(bool b);
	void setSavePackingButtons(bool b);
	void setSaved(bool b);
	void setRotationButton(bool b);

    //Canvas:
    cg3::Point2i canvasSize() const;
	// AbstractMainWindow interface
	void updateCanvas();
	void fitSceneCanvas();

    //DrawableObjects for the Canvas
    void pushDrawableObject(
            const cg3::DrawableObject * obj,
            std::string checkBoxName = "",
            bool checkBoxChecked = true,
            bool closeButtonVisible = false);
	void pushDrawableObject(const std::shared_ptr<const cg3::DrawableObject> &ptr,
            std::string checkBoxName = "",
            bool checkBoxChecked = true,
            bool closeButtonVisible = false);
    bool deleteDrawableObject(const cg3::DrawableObject * obj);
	bool deleteDrawableObject(const std::shared_ptr<const cg3::DrawableObject> &ptr);
    void setDrawableObjectVisibility(const cg3::DrawableObject * obj, bool visible);
	void setDrawableObjectVisibility(const std::shared_ptr<const cg3::DrawableObject> &ptr, bool visible);
    bool containsDrawableObject(const cg3::DrawableObject* obj);
	bool containsDrawableObject(const std::shared_ptr<const cg3::DrawableObject> &ptr);
    bool refreshDrawableObject(const cg3::DrawableObject* obj);
	bool refreshDrawableObject(const std::shared_ptr<const cg3::DrawableObject> &ptr);
    bool setDrawableObjectName(
            const cg3::DrawableObject* obj,
            const std::string& newName);
    bool setDrawableObjectName(
			const std::shared_ptr<const cg3::DrawableObject> &ptr,
            const std::string& newName);
    std::string nameOfDrawableObject(const cg3::DrawableObject* obj) const;
	std::string nameOfDrawableObject(const std::shared_ptr<const cg3::DrawableObject> &ptr) const;
    std::vector<const cg3::DrawableObject*> selectedDrawableObjects() const;
	const cg3::DrawableObject* selectedDrawableObject() const;

    //Debug Objects
    void enableDebugObjects();
    void disableDebugObjects();
    void toggleDebugObjects();
    void toggleUnitBox();

    //Window Options:
    void setFullScreen(bool);
    void keyPressEvent(QKeyEvent * event); //event options for keys pressed

signals:

    /**
     * @brief undoEvent
     * Conntect your slot with this signal in ordert to execute an action when the
     * user uses CTRL+Z in the MainWindow
     */
    void undoEvent();

    /**
     * @brief redoEvent
     * Conntect your slot with this signal in ordert to execute an action when the
     * user uses MAIUSC+CTRL+Z in the MainWindow
     */
    void redoEvent();

private slots:	

    //Menu Actions
	void on_actionLoad_Mesh_triggered();
	void on_actionLoad_SAM_Project_triggered();
	void on_actionSave_SAM_Project_triggered();
	void on_actionSave_SAM_Project_As_triggered();
	void on_actionSave_Decomposition_triggered();
	void on_actionSave_Packing_triggered();
    void on_actionSave_Snapshot_triggered();
    void on_actionShow_Axis_triggered();
    void on_actionFull_Screen_toggled(bool arg1);
    void on_actionUpdate_Canvas_triggered();
    void on_actionFit_Scene_triggered();
    void on_actionChange_Background_Color_triggered();
    void on_actionSave_Point_Of_View_triggered();
    void on_actionLoad_Point_of_View_triggered();
    void on_actionShow_Hide_Dock_Widget_triggered();
    void on_actionLoad_Point_Of_View_from_triggered();
    void on_actionSave_Point_Of_View_as_triggered();
    void on_actionShow_Hide_DrawList_triggered();
    void on_actionReset_Point_of_View_triggered();
    void on_actionPerspective_Orthographic_Camera_Mode_triggered();
	void on_actionShow_Box_triggered();

	void on_loadToolButton_clicked();
	void on_loadSAMToolButton_clicked();
	void on_saveSAMToolButton_clicked();
	void on_saveDecompositionToolButton_clicked();
	void on_savePackingToolButton_clicked();
	void on_orthoToolButton_toggled(bool b);
	void on_showAxisToolButton_toggled(bool b);
	void on_showBoxToolButton_toggled(bool b);
	void on_rotationToolButton_toggled(bool b);
	void on_resetRotationToolButton_clicked();

	// QWidget interface
protected:
	void closeEvent(QCloseEvent *event);

private:

    struct ContainerProperties {
        QFrame* frame;
        std::vector<QCheckBox*> checkBoxes;
    };

    // GUI
    //
    //Ui::MainWindow* ui;
    internal::UiMainWindowRaiiWrapper* ui;
    QVBoxLayout* scrollAreaLayout;
    cg3::viewer::LoaderSaver povLS;
    QSpacerItem* m_spacer;
	HFGui* hfFrame;

    // Mesh Stack
    //
	std::map<const cg3::DrawableObject*, cg3::viewer::DrawableObjectDrawListManager*> mapDrawListManagers;
    std::set<std::shared_ptr<const cg3::DrawableObject> > sharedDrawableObjects;
	bool saved;
	std::string fileSAM;

public:

	cg3::viewer::GLCanvas& canvas; /** @brief Public member of type cg3::viewer::GLCanvas that allows
                          to manage the canvas contained in the MainWindow. */
};

#endif // HFMAINWINDOW_H

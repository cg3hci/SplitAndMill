/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @author Marco Livesu (marco.livesu@gmail.com)
 */

#include "hfmainwindow.h"
#include "ui_hfmainwindow.h"

#include <QScrollArea>
#include <QSpacerItem>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QColorDialog>
#include <QDockWidget>
#include <QToolBox>
#include <QFrame>

#include <cg3/viewer/interfaces/drawable_container.h>
#include <cg3/viewer/interfaces/drawable_mesh.h>
#include <cg3/viewer/internal/drawable_mesh_drawlist_manager.h>
#include <cg3/viewer/internal/drawable_container_drawlist_manager.h>
#include <cg3/viewer/internal/drawable_object_drawlist_manager.h>

#include <cg3/utilities/cg3_config_folder.h>
#include <cg3/utilities/string.h>
#include <cg3/utilities/timer.h>

#include "hf_gui.h"


namespace internal {
class UiMainWindowRaiiWrapper : public Ui::HFMainWindow
{
public:
    UiMainWindowRaiiWrapper(QMainWindow *MainWindow)
    {
        setupUi(MainWindow);
    }		
};
} //namespace cg3::viewer::internal

/**
 * @brief Constructor that creates and initializes all the members of the HFMainWindow,
 * setting up the canvas and linking it to the scroll area that will contain the
 * checkboxes associtated to the DrawableObjects contained in the canvas.
 */
HFMainWindow::HFMainWindow(QWidget* parent) :
		AbstractMainWindow(parent),
        ui(new internal::UiMainWindowRaiiWrapper(this)),
		hfFrame(nullptr),
		saved(true),
		canvas(*ui->glCanvas)
{
    scrollAreaLayout = new QVBoxLayout(ui->scrollArea);
    ui->scrollArea->setLayout(scrollAreaLayout);

    m_spacer = new QSpacerItem(40, 20, QSizePolicy::Minimum, QSizePolicy::Expanding);
    scrollAreaLayout->addItem(m_spacer);

    povLS.addSupportedExtension("cg3pov");

	QMainWindow::showMaximized();

    canvas.setSnapshotQuality(100);
    canvas.setSnapshotFormat("PNG");

	//ui->dockDrawList->setVisible(false);

	ui->saveSAMToolButton->setEnabled(false);
	ui->savePackingToolButton->setEnabled(false);
	ui->saveDecompositionToolButton->setEnabled(false);
	ui->actionSave_SAM_Project->setEnabled(false);
	ui->actionSave_Decomposition->setEnabled(false);
	ui->actionSave_Packing->setEnabled(false);

	setWindowTitle("Split and Mill - Untilted Project");
}

HFMainWindow::~HFMainWindow()
{
	delete ui;
}

void HFMainWindow::setWidget(HFGui *frame)
{
	ui->dockToolBox->setWidget(frame);
	hfFrame = frame;
}

void HFMainWindow::setLoadedButtons(bool b)
{
	ui->actionSave_SAM_Project->setEnabled(b);
	ui->saveSAMToolButton->setEnabled(b);
	setSaveDecompositionButtons(b && hfFrame->decompositionComputed());
	setSavePackingButtons(b && hfFrame->packingComputed());
}

void HFMainWindow::setSaveDecompositionButtons(bool b)
{
	ui->saveDecompositionToolButton->setEnabled(b);
	ui->actionSave_Decomposition->setEnabled(b);
}

void HFMainWindow::setSavePackingButtons(bool b)
{
	ui->savePackingToolButton->setEnabled(b);
	ui->actionSave_Packing->setEnabled(b);
}

void HFMainWindow::setSaved(bool b)
{
	saved = b;
	std::string pname;
	if (fileSAM != "") pname = cg3::filenameWithoutExtension(fileSAM);
	else pname = "Untilted Project";
	if (saved) {
		setWindowTitle("Split and Mill - " + QString::fromStdString(pname));
	}
	else {
		setWindowTitle("Split and Mill - " + QString::fromStdString(pname) + "*");
	}
}

void HFMainWindow::setRotationButton(bool b)
{
	ui->rotationToolButton->setEnabled(b);
}

/**
 * @brief Returns the sizes of the Canvas as number of pixels.
 */
cg3::Point2i HFMainWindow::canvasSize() const
{
	return cg3::Point2i(canvas.width(), canvas.height());
}

void HFMainWindow::updateCanvas()
{
	canvas.update();
}

void HFMainWindow::fitSceneCanvas()
{
	canvas.fitScene();
}

/**
 * @brief Adds a new DrawableObject to the canvas and links it to a new checkbox in the
 * ScrollArea, and updates automatically the scene.
 *
 * @param obj: the DrawableObject
 * @param checkBoxName: a name associated to the DrawableObject
 * @param checkBoxChecked: true if the object will be visible, false otherwise
 */
void HFMainWindow::pushDrawableObject(
		const cg3::DrawableObject* obj,
        std::string checkBoxName,
        bool checkBoxChecked,
        bool closeButtonVisible)
{
    if (obj != nullptr && !canvas.containsDrawableObject(obj)) {
        canvas.pushDrawableObject(obj, checkBoxChecked);
		cg3::viewer::DrawableObjectDrawListManager* manager =
				new cg3::viewer::DrawableObjectDrawListManager(this, obj, checkBoxName, checkBoxChecked, closeButtonVisible);
        mapDrawListManagers[obj] = manager;

        scrollAreaLayout->addWidget(manager);

        scrollAreaLayout->removeItem(m_spacer);
        scrollAreaLayout->addItem(m_spacer);
        canvas.update();
    }
}

void HFMainWindow::pushDrawableObject(
		const std::shared_ptr<const cg3::DrawableObject> &ptr,
        std::string checkBoxName,
        bool checkBoxChecked,
        bool closeButtonVisible)
{
    if (!containsDrawableObject(ptr)){
        sharedDrawableObjects.insert(ptr);
        pushDrawableObject(ptr.get(), checkBoxName, checkBoxChecked, closeButtonVisible);
    }
}

/**
 * @brief Removes the DrawableObject from the canvas and the relative checkbox from the
 * HFMainWindow.
 *
 * @param obj: the object that will be removed from the canvas
 */
bool HFMainWindow::deleteDrawableObject(const cg3::DrawableObject* obj)
{
    if (obj != nullptr && mapDrawListManagers.find(obj) != mapDrawListManagers.end()){
        canvas.deleteDrawableObject(obj);
        //mapDrawListManagers[obj]->deleteSubManager();
        scrollAreaLayout->removeWidget(mapDrawListManagers[obj]);
        delete mapDrawListManagers[obj];
        mapDrawListManagers.erase(obj);
        canvas.update();
        return true;
    }
	else {
		if (canvas.containsDrawableObject(obj)){
			canvas.deleteDrawableObject(obj);
			canvas.update();
		}
	}
    return false;
}

bool HFMainWindow::deleteDrawableObject(const std::shared_ptr<const cg3::DrawableObject> &ptr)
{
    bool b = deleteDrawableObject(ptr.get());
    sharedDrawableObjects.erase(ptr);
    return b;
}

/**
 * @brief Sets the visibility/non visibility of a DrawableObject, checking/unchecking its
 * checkbox accordingly if it is in the list of DrawableObjects of the HFMainWindow.
 */
void HFMainWindow::setDrawableObjectVisibility(const cg3::DrawableObject* obj, bool visible)
{
    if (mapDrawListManagers.find(obj) != mapDrawListManagers.end()){
		if (obj->isVisible() != visible)
			mapDrawListManagers[obj]->setDrawableObjectVisibility(visible);

    }
	canvas.setDrawableObjectVisibility(obj, visible);
	canvas.update();
}

void HFMainWindow::setDrawableObjectVisibility(const std::shared_ptr<const cg3::DrawableObject> &ptr, bool visible)
{
    setDrawableObjectVisibility(ptr.get(), visible);
}

/**
 * @brief Returns true if the input DrawableObject is already drawn in the canvas.
 */
bool HFMainWindow::containsDrawableObject(const cg3::DrawableObject* obj)
{
    return (mapDrawListManagers.find(obj) != mapDrawListManagers.end());
}

bool HFMainWindow::containsDrawableObject(const std::shared_ptr<const cg3::DrawableObject> &ptr)
{
    return sharedDrawableObjects.find(ptr) != sharedDrawableObjects.end();
}

/**
 * @brief Refreshes the visibility of a DrawableObject if it was assigned.
 * Sets the properties of the user interface in the DrawableObject.
 * @todo Manage PickableObject and DrawableContainer..
 */
bool HFMainWindow::refreshDrawableObject(const cg3::DrawableObject* obj)
{
    if (mapDrawListManagers.find(obj) != mapDrawListManagers.end()) {

		const cg3::PickableObject* pobj = dynamic_cast<const cg3::PickableObject*>(obj);
        if (pobj) {
            canvas.deleteDrawableObject(obj);
            canvas.pushDrawableObject(obj, obj->isVisible());
            canvas.update();
        }
        mapDrawListManagers[obj]->updateObjectProperties();

        return true;
    }
    else {
        return false;
    }
}

bool HFMainWindow::refreshDrawableObject(const std::shared_ptr<const cg3::DrawableObject> &ptr)
{
    return refreshDrawableObject(ptr.get());
}

bool HFMainWindow::setDrawableObjectName(const cg3::DrawableObject* obj, const std::string& newName)
{
    if (mapDrawListManagers.find(obj) != mapDrawListManagers.end()){
        mapDrawListManagers[obj]->setDrawableObjectName(newName);
        return true;
    }
    return false;
}

bool HFMainWindow::setDrawableObjectName(const std::shared_ptr<const cg3::DrawableObject> &ptr, const std::string &newName)
{
    return setDrawableObjectName(ptr.get(), newName);
}

std::string HFMainWindow::nameOfDrawableObject(const cg3::DrawableObject* obj) const
{
    auto it = mapDrawListManagers.find(obj);
    if (it != mapDrawListManagers.end()){
        return it->second->drawableObjectName();
    }
    else
        return "";
}

std::string HFMainWindow::nameOfDrawableObject(const std::shared_ptr<const cg3::DrawableObject> &ptr) const
{
    return nameOfDrawableObject(ptr.get());
}

/**
 * @brief Returns a vector of all the selected DrawableObjects of the DrawList.
 * @return the vector of selected objects.
 */
std::vector<const cg3::DrawableObject*> HFMainWindow::selectedDrawableObjects() const
{
	std::vector<const cg3::DrawableObject*> vec;
	for (const std::pair<const cg3::DrawableObject*, cg3::viewer::DrawableObjectDrawListManager*>& p :
         mapDrawListManagers){
        if (p.second->isSelected()){
            vec.push_back(p.first);
        }
        if (p.second->isContainer()){
			std::vector<const cg3::DrawableObject*> tmp = p.second->containedSelectedObjects();
            vec.insert(vec.end(), tmp.begin(), tmp.end());
        }
    }
    return vec;
}

/**
 * @brief Returns the only DrawableObject which is selected, nullptr if none or more than
 * one object is selected.
 * @return the selected DrawableObject (if and ONLY if there is one selected DrawableObject)
 */
const cg3::DrawableObject* HFMainWindow::selectedDrawableObject() const
{
	const cg3::DrawableObject* obj = nullptr;
	for (const std::pair<const cg3::DrawableObject*, cg3::viewer::DrawableObjectDrawListManager*>& p :
         mapDrawListManagers){
        if (p.second->isSelected()){
            if (obj == nullptr){
                obj = p.first;
            }
            else {
                return nullptr;
            }
        }
        if (p.second->isContainer()){
			std::vector<const cg3::DrawableObject*> tmp = p.second->containedSelectedObjects();
            if (tmp.size() > 0){
                if (obj == nullptr && tmp.size() == 1){
                    obj = tmp[0];
                }
                else {
                    return nullptr;
                }
            }
        }
    }
    return obj;
}

/**
 * @brief Sets the full screen mode to the HFMainWindow according to the boolean parameter.
 */
void HFMainWindow::setFullScreen(bool b)
{
    canvas.setFullScreen(b);
    if (!b)
        showMaximized();
}

/**
 * @brief Manages a Key Event and executes relative operations or emits signals.
 */
void HFMainWindow::keyPressEvent(QKeyEvent * event)
{
    if(event->matches(QKeySequence::Undo))
        emit(undoEvent());
    if (event->matches(QKeySequence::Redo))
		emit(redoEvent());
}

void HFMainWindow::on_actionLoad_Mesh_triggered()
{
	bool load = true;
	if (!saved){
		std::string pname;
		if (pname != "") pname = cg3::filenameWithoutExtension(fileSAM);
		else pname = "Untilted Project";
		QMessageBox* box = new QMessageBox(this);
		box->setWindowTitle("");
		box->setText("Save changes to " + QString::fromStdString(pname) + " before loading new mesh?");
		box->addButton(QMessageBox::No);
		box->button(QMessageBox::No)->setText("No");

		box->addButton(QMessageBox::Ok);
		box->button(QMessageBox::Ok)->setText("Yes");

		box->addButton(QMessageBox::Cancel);
		box->button(QMessageBox::Cancel)->setText("Cancel");

		box->setEscapeButton(QMessageBox::No);
		box->setDefaultButton(QMessageBox::Cancel);
		int ret = box->exec();
		if (ret == QMessageBox::No)
			load = true;
		else if (ret == QMessageBox::Ok){
			ui->actionSave_SAM_Project->trigger();
			if (!saved)
				load = false;
		}
		else {
			load = false;
		}
	}
	if (load){
		bool b = hfFrame->loadMesh();
		setRotationButton(b);
		setLoadedButtons(b);
		fileSAM = "";
		setSaved(false);
	}
}

void HFMainWindow::on_actionLoad_SAM_Project_triggered()
{
	bool load = true;
	if (!saved){
		std::string pname;
		if (pname != "") pname = cg3::filenameWithoutExtension(fileSAM);
		else pname = "Untilted Project";
		QMessageBox* box = new QMessageBox(this);
		box->setWindowTitle("");
		box->setText("Save changes to " + QString::fromStdString(pname) + " before loading new project?");
		box->addButton(QMessageBox::No);
		box->button(QMessageBox::No)->setText("No");

		box->addButton(QMessageBox::Ok);
		box->button(QMessageBox::Ok)->setText("Yes");

		box->addButton(QMessageBox::Cancel);
		box->button(QMessageBox::Cancel)->setText("Cancel");

		box->setEscapeButton(QMessageBox::No);
		box->setDefaultButton(QMessageBox::Cancel);
		int ret = box->exec();
		if (ret == QMessageBox::No)
			load = true;
		else if (ret == QMessageBox::Ok){
			ui->actionSave_SAM_Project->trigger();
			if (!saved)
				load = false;
		}
		else {
			load = false;
		}
	}
	if (load){
		bool b = hfFrame->loadSAM(fileSAM);
		if (b) {
			setSaved(true);
			setLoadedButtons(true);
		}
	}
}

void HFMainWindow::on_actionSave_SAM_Project_triggered()
{
	if (fileSAM == "")
		ui->actionSave_SAM_Project_As->trigger();
	else{
		bool b = hfFrame->saveSAM(fileSAM);
		setSaved(b);
	}
}

void HFMainWindow::on_actionSave_SAM_Project_As_triggered()
{
	bool b = hfFrame->saveSAMAs(fileSAM);
	setSaved(b);
}

void HFMainWindow::on_actionSave_Decomposition_triggered()
{
	hfFrame->saveDecomposition();
}

void HFMainWindow::on_actionSave_Packing_triggered()
{
	hfFrame->savePacking();
}

void HFMainWindow::on_actionSave_Snapshot_triggered()
{
    ui->glCanvas->saveSnapshot(QString(), false);
}

void HFMainWindow::on_actionShow_Axis_triggered()
{
    ui->glCanvas->toggleAxisIsDrawn();
}

void HFMainWindow::on_actionFull_Screen_toggled(bool arg1)
{
    setFullScreen(arg1);
}

void HFMainWindow::on_actionUpdate_Canvas_triggered()
{
    canvas.update();
}

void HFMainWindow::on_actionFit_Scene_triggered()
{
    canvas.fitScene();
}

void HFMainWindow::on_actionChange_Background_Color_triggered()
{
    QColor color = QColorDialog::getColor(Qt::white, this);

    canvas.setBackgroundColor(color);
    canvas.update();
}

void HFMainWindow::on_actionSave_Point_Of_View_triggered()
{
    canvas.savePointOfView();
}

void HFMainWindow::on_actionLoad_Point_of_View_triggered()
{
    canvas.loadPointOfView();
}

void HFMainWindow::on_actionShow_Hide_Dock_Widget_triggered()
{
    if (ui->dockToolBox->isHidden())
        ui->dockToolBox->show();
    else
        ui->dockToolBox->hide();
}

void HFMainWindow::on_actionLoad_Point_Of_View_from_triggered()
{
    std::string s = povLS.loadDialog("Open Point Of View");
    if (s != ""){
        canvas.loadPointOfView(s);
    }
}

void HFMainWindow::on_actionSave_Point_Of_View_as_triggered()
{
    std::string s = povLS.saveDialog("Save Point Of View");
    if (s != ""){
        canvas.savePointOfView(s);
    }
}

void HFMainWindow::on_actionShow_Hide_DrawList_triggered()
{
    if (ui->dockDrawList->isHidden())
        ui->dockDrawList->show();
    else
        ui->dockDrawList->hide();
}

void HFMainWindow::on_actionReset_Point_of_View_triggered()
{
    canvas.resetPointOfView();
}

void HFMainWindow::on_actionPerspective_Orthographic_Camera_Mode_triggered()
{
    canvas.toggleCameraType();
    canvas.update();
}

void HFMainWindow::on_actionShow_Box_triggered()
{
	hfFrame->drawBox();
}

void HFMainWindow::on_loadToolButton_clicked()
{
	ui->actionLoad_Mesh->trigger();
}

void HFMainWindow::on_loadSAMToolButton_clicked()
{
	ui->actionLoad_SAM_Project->trigger();
}

void HFMainWindow::on_saveSAMToolButton_clicked()
{
	ui->actionSave_SAM_Project->trigger();
}

void HFMainWindow::on_saveDecompositionToolButton_clicked()
{
	ui->actionSave_Decomposition->trigger();
}

void HFMainWindow::on_savePackingToolButton_clicked()
{
	ui->actionSave_Packing->trigger();
}

void HFMainWindow::on_orthoToolButton_toggled(bool b)
{
	if (b != ui->glCanvas->isOrthographicCamera())
		ui->actionPerspective_Orthographic_Camera_Mode->trigger();
}

void HFMainWindow::on_showAxisToolButton_toggled(bool b)
{
	if (b != ui->glCanvas->axisIsDrawn())
		ui->actionShow_Axis->trigger();
}

void HFMainWindow::on_showBoxToolButton_toggled(bool b)
{
	if (b != hfFrame->boxIsDrawn())
		ui->actionShow_Box->trigger();
}

void HFMainWindow::on_rotationToolButton_toggled(bool b)
{
	hfFrame->enableManualRotation(b);
	ui->resetRotationToolButton->setEnabled(b);
}

void HFMainWindow::on_resetRotationToolButton_clicked()
{
	hfFrame->resetRotation();
	ui->rotationToolButton->toggle();
	ui->resetRotationToolButton->setEnabled(false);
}

void HFMainWindow::closeEvent(QCloseEvent *event)
{
	if (!saved){
		std::string pname;
		if (pname != "") pname = cg3::filenameWithoutExtension(fileSAM);
		else pname = "Untilted Project";
		QMessageBox* box = new QMessageBox(this);
		box->setWindowTitle("");
		box->setText("Save changes to " + QString::fromStdString(pname) + " before closing?");
		box->addButton(QMessageBox::No);
		box->button(QMessageBox::No)->setText("Close Without Saving");

		box->addButton(QMessageBox::Ok);
		box->button(QMessageBox::Ok)->setText("Save");

		box->addButton(QMessageBox::Cancel);
		box->button(QMessageBox::Cancel)->setText("Cancel");
		box->setEscapeButton(QMessageBox::Cancel);
		box->setDefaultButton(QMessageBox::Ok);
		int ret = box->exec();
		if (ret == QMessageBox::Cancel)
			event->ignore();
		else if (ret == QMessageBox::Ok){
			ui->actionSave_SAM_Project->trigger();
			if (!saved)
				event->ignore();
		}
		else {
			QMainWindow::closeEvent(event);
		}
	}
	if (saved)
		QMainWindow::closeEvent(event);
}

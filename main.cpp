#include <gui/hfmainwindow.h>
//#include <cg3/viewer/managers/booleans_manager.h>
#include <gui/hf_gui.h>

int main(int argc, char *argv[]) {

    CG3_SUPPRESS_WARNING(argc);
    CG3_SUPPRESS_WARNING(argv);

    QApplication app(argc, argv);
	HFMainWindow gui;  //Main window, it contains QGLViewer canvas
	gui.canvas.update();


	HFGui man(&gui);
	gui.setWidget(&man);

	//Show the GUI
    gui.show();

    return app.exec();
}

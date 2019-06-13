#include <cg3/viewer/mainwindow.h>
#include <gui/hf_gui.h>

int main(int argc, char *argv[]) {

    CG3_SUPPRESS_WARNING(argc);
    CG3_SUPPRESS_WARNING(argv);

    QApplication app(argc, argv);
    cg3::viewer::MainWindow gui;  //Main window, it contains QGLViewer canvas

	HFGui man(&gui);
	gui.addManager(&man, "HF Decomposition");

	//Show the GUI
    gui.canvas.update();
    gui.show();

    return app.exec();
}

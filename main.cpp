#include <cg3/viewer/mainwindow.h>
//#include <cg3/viewer/managers/booleans_manager.h>
#include <gui/hf_gui.h>

int main(int argc, char *argv[]) {

    CG3_SUPPRESS_WARNING(argc);
    CG3_SUPPRESS_WARNING(argv);

    QApplication app(argc, argv);
    cg3::viewer::MainWindow gui;  //Main window, it contains QGLViewer canvas

	HFGui man(&gui);
	gui.addManager(&man, "HF Decomposition");

	//cg3::viewer::BooleansManager bm(&gui);
	//gui.addManager(&bm, "booleans");

//	std::ifstream t("/mnt/Dati/Drive/Research/Repos/Projects/HFDecompositionGUI/cg3lib/cg3/viewer/internal/darkstyle/darkstyle.qss");
//	std::string str;

//	t.seekg(0, std::ios::end);
//	str.reserve(t.tellg());
//	t.seekg(0, std::ios::beg);

//	str.assign((std::istreambuf_iterator<char>(t)),
//				std::istreambuf_iterator<char>());

//	app.setStyleSheet(str.c_str());

	//Show the GUI
    gui.canvas.update();
    gui.show();

    return app.exec();
}

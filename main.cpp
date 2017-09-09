///Test cg3 core module
#ifdef CG3_CORE_DEFINED
#include <cg3/data_structures/arrays.h>
#include <cg3/geometry/bounding_box.h>
#include <cg3/utilities/comparators.h>
#include <cg3/utilities/timer.h>
#endif

///Test cg3 viewer module
#ifdef CG3_VIEWER_DEFINED
#include <cg3/viewer/interfaces/drawable_object.h>
#include <cg3/viewer/interfaces/pickable_object.h>
#include <cg3/viewer/mainwindow.h>
#include <cg3/viewer/managers/window_manager/window_manager.h>
#endif

///Test cg3 dcel module
#ifdef CG3_DCEL_DEFINED
#include <cg3/meshes/dcel/dcel.h>
//dcel manager
#ifdef CG3_VIEWER_DEFINED
#include <cg3/viewer/managers/dcel_manager/dcel_manager.h>
#endif
#endif

//Test eigenmesh module
#ifdef CG3_EIGENMESH_DEFINED
#include <cg3/meshes/eigenmesh/eigenmesh.h>
///Test trimeshviewer.pri: uncomment to test trimesh module
#ifdef CG3_VIEWER_DEFINED
#include <cg3/viewer/managers/eigenmesh_manager/eigenmesh_manager.h>
#ifdef CG3_CGAL_DEFINED
#ifdef CG3_LIBIGL_DEFINED
#include <cg3/viewer/managers/booleans_manager/booleans_manager.h>
#endif
#endif
#endif
#endif

int main(int argc, char *argv[]) {

    ///Test viewer.pri:
    #ifdef CG3_VIEWER_DEFINED
    QApplication app(argc, argv);

    MainWindow gui;  // finestra principale, contiene la canvas di QGLViewer


    WindowManager wm(&gui); // Creo un window manager e lo aggiungo alla mainwindow
    int id = gui.addManager(&wm, "Window");

    //Test eigenmeshmanager.pri
    #ifdef CG3_EIGENMESH_DEFINED
    EigenMeshManager em(&gui);
    id = gui.addManager(&em, "EigenMesh");


    //Test booleansmanager.pri
    #if defined(CG3_LIBIGL_DEFINED) && defined(CG3_CGAL_DEFINED)
    BooleansManager bm(&gui);
    id = gui.addManager(&bm, "Booleans");
    #endif
    #endif

    //Test dcelmanager.pri
    #ifdef CG3_DCEL_DEFINED
    DcelManager dm(&gui);
    id = gui.addManager(&dm, "Dcel");
    #endif

    gui.setCurrentIndexToolBox(id); // il window manager sarà quello visualizzato di default
    gui.updateGlCanvas();
    gui.show();

    return app.exec();
    #else
    std::cout << "Hello World!" << std::endl;
    return 0;
    #endif
}

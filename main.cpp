//If core defined
#ifdef CG3_CORE_DEFINED

//Viewer defined
#ifdef CG3_VIEWER_DEFINED
#include <cg3/viewer/mainwindow.h>

//DCEL module defined
#ifdef CG3_DCEL_DEFINED
#include <cg3/viewer/managers/dcel_manager.h>
#endif

//Eigenmesh module defined
#ifdef CG3_EIGENMESH_DEFINED
#include <cg3/viewer/managers/eigenmesh_manager.h>

//CGAL and LIBIGL defined
#ifdef CG3_CGAL_DEFINED
#ifdef CG3_LIBIGL_DEFINED
#include <cg3/viewer/managers/booleans_manager.h>
#endif
#endif
#endif

//If viewer not defined
#else
#include <iostream>
#define CG3_SUPPRESS_WARNING(p) (void)p
#endif

//If core not defined
#else
#include <iostream>
#define CG3_SUPPRESS_WARNING(p) (void)p
#endif

int main(int argc, char *argv[]) {

    CG3_SUPPRESS_WARNING(argc);
    CG3_SUPPRESS_WARNING(argv);

//If cg3 is not included
#ifdef CG3_CORE_DEFINED

//If viewer module is included
#ifdef CG3_VIEWER_DEFINED

    QApplication app(argc, argv);
    cg3::viewer::MainWindow gui;  //Main window, it contains QGLViewer canvas

//If EigenMesh module is included
#ifdef CG3_EIGENMESH_DEFINED

    //Add EigenMesh manager
    cg3::viewer::EigenMeshManager em(&gui);
    int idEigen = gui.addManager(&em, "EigenMesh");
    CG3_SUPPRESS_WARNING(idEigen);


//If LIBIGL and CGAL are included
#if defined(CG3_LIBIGL_DEFINED) && defined(CG3_CGAL_DEFINED)

    //Add boolean manager
    cg3::viewer::BooleansManager bm(&gui);
    int idBoolean = gui.addManager(&bm, "Booleans");
    CG3_SUPPRESS_WARNING(idBoolean);
#endif

#endif

//If DCEL module is included
#ifdef CG3_DCEL_DEFINED

    //Add DCEL manager
    cg3::viewer::DcelManager dm(&gui);
    int idDcel = gui.addManager(&dm, "Dcel");
    CG3_SUPPRESS_WARNING(idDcel);

#endif

    //Set the window manager as the default one
    gui.setCurrentManager(0);

    //Show the GUI
    gui.canvas.update();
    gui.show();

    return app.exec();

//If viewer module is not included
#else

    std::cout << "Impossible to open the window: the module ''viewer'' has not been included." << std::endl;
    return 0;

#endif

//If cg3 core is not included
#else

    std::cout << "Impossible to load the library cg3: the module ''core'' has not been included." << std::endl;
    return 0;

#endif

}

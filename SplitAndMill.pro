# Debug configuration
CONFIG(debug, debug|release){
    DEFINES += DEBUG
}

# Release configuration
CONFIG(release, debug|release){
    DEFINES -= DEBUG

    # Uncomment next line if you want to ignore asserts and got a more optimized binary
    #CONFIG += FINAL_RELEASE
}

CONFIG+=static

# Final release optimization
FINAL_RELEASE {
    unix:!macx{
        QMAKE_CXXFLAGS_RELEASE -= -g -O2
        QMAKE_CXXFLAGS += -DNDEBUG
    }
}

DESTDIR = release

# cg3lib works with c++11
CONFIG += c++11

# Cg3lib configuration
VCGLIB_PATH = $$PWD/vcglib/
LIBIGL_PATH = $$PWD/libigl/
CONFIG += CG3_CORE CG3_VIEWER CG3_DATA_STRUCTURES CG3_ALGORITHMS CG3_MESHES CG3_CGAL CG3_LIBIGL CG3_VCGLIB
CONFIG += CG3_WIN_MSVC
include (cg3lib/cg3.pri)
message($$MODULES)

HEADERS += \
    data_structures/hf_box.h \
    data_structures/hf_engine.h \
    data_structures/high_frequencies_restore.h \
    data_structures/packing.h \
    data_structures/user_action.h \
    gui/arrow.h \
    gui/guides.h \
    gui/hf_engine_thread.h \
    gui/hf_gui.h \
    gui/hfmainwindow.h \
    gui/manipulable_boundingbox.h \
    gui/manipulable_sphere.h \
    gui/rotatable_mesh.h \
    lib/packing/binpack2d.h


SOURCES += \
    data_structures/hf_box.cpp \
    data_structures/hf_engine.cpp \
    data_structures/high_frequencies_restore.cpp \
    data_structures/packing.cpp \
    data_structures/user_action.cpp \
    gui/guides.cpp \
    gui/hf_engine_thread.cpp \
    gui/hf_gui.cpp \
    gui/hfmainwindow.cpp \
    gui/manipulable_boundingbox.cpp \
    gui/manipulable_sphere.cpp \
    gui/rotatable_mesh.cpp \
    main.cpp

FORMS += \
    gui/hf_gui.ui \
    gui/hfmainwindow.ui

RESOURCES += \
    res/res.qrc


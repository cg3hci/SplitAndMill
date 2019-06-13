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

# Final release optimization
FINAL_RELEASE {
    unix:!macx{
        QMAKE_CXXFLAGS_RELEASE -= -g -O2
        QMAKE_CXXFLAGS += -Os -DNDEBUG
    }
}

# cg3lib works with c++11
CONFIG += c++11

# Cg3lib configuration. Available options:
#
#   CG3_ALL                 -- All the modules
#
#   CG3_CORE                -- Core of the library. Geometry primitives and utilities
#   CG3_VIEWER              -- Module containing utilities for creating viewers (Qt and OpenGL)
#
#   CG3_DATA_STRUCTURES     -- Various data structure
#   CG3_ALGORITHMS          -- Various algorithms
#
#   CG3_MESHES              -- Mesh data structures
#
#   CG3_CGAL                -- CGAL interface
#   CG3_LIBIGL              -- libIGL interface
#   CG3_CINOLIB             -- CinoLib interface
#
# Example:  CONFIG += CG3_CORE CG3_VIEWER CG3_DATA_STRUCTURES CG3_MESHES
CONFIG += CG3_ALL
CONFIG -= CG3_CINOLIB

# Include the chosen modules
include (cg3lib/cg3.pri)
message($$MODULES)

HEADERS += \
    gui/hf_gui.h \
    gui/manipulableboundingbox.h \
    gui/manipulablesphere.h

SOURCES += \
    gui/hf_gui.cpp \
    gui/manipulableboundingbox.cpp \
    gui/manipulablesphere.cpp \
    main.cpp

FORMS += \
    gui/hf_gui.ui


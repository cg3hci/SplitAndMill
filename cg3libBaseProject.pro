include(cg3lib/cg3.pri)

message(Included modules: $$MODULES)
FINAL_RELEASE {
    message(Final Release!)
}

SOURCES += \
    main.cpp

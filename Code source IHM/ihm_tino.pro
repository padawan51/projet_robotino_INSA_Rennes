QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ihm_tino
TEMPLATE = app

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    acquisitionthread.cpp \
    cameras.cpp \
    cameraviewbutton.cpp \
    checknodesthread.cpp \
    chronothread.cpp \
    correctordialog.cpp \
    css.cpp \
    main.cpp \
    mainwindow.cpp \
    measurement.cpp \
    moverobotthread.cpp \
    mymath.cpp \
    path.cpp \
    pathplanning.cpp \
    robot.cpp \
    robotcontrollerthread.cpp \
    robotpositionthread.cpp \
    thresholddialog.cpp \
    topographicmap.cpp \
    utils.cpp \
    workingarea.cpp

HEADERS += \
    acquisitionthread.h \
    cameras.h \
    cameraviewbutton.h \
    checknodesthread.h \
    chronothread.h \
    correctordialog.h \
    css.h \
    mainwindow.h \
    measurement.h \
    moverobotthread.h \
    mymath.h \
    path.h \
    pathplanning.h \
    robot.h \
    robotcontrollerthread.h \
    robotpositionthread.h \
    thresholddialog.h \
    topographicmap.h \
    types.h \
    utils.h \
    workingarea.h

FORMS += \
    correctordialog.ui \
    mainwindow.ui \
    thresholddialog.ui

include(3rdparty/qtxlsx/QtXlsxWriter/src/xlsx/qtxlsx.pri)

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

LIBS += -LC:\OpenCV\opencv\build\install\x64\vc16\lib \
        #-lopencv_core455d \
        #-lopencv_highgui455d \
        #-lopencv_imgproc455d \
        #-lopencv_imgcodecs455d \
        #-lopencv_videoio455d \
        #-lopencv_video455d \
        #-lopencv_calib3d455d \
        #-lopencv_photo455d \
        #-lopencv_features2d455d \
        -lopencv_core455 \
        -lopencv_highgui455 \
        -lopencv_imgproc455 \
        -lopencv_imgcodecs455 \
        -lopencv_videoio455 \
        -lopencv_video455 \
        -lopencv_calib3d455 \
        -lopencv_photo455 \
        -lopencv_features2d455

INCLUDEPATH += C:\OpenCV\opencv\build\install\include
DEPENDPATH += C:\OpenCV\opencv\build\install\include

DEFINES += IMG_960x540_W=960 #pixels
DEFINES += IMG_960x540_H=540 #pixels
DEFINES += IMG_800x600_W=800 #pixels
DEFINES += IMG_800x600_H=600 #pixels
DEFINES += IMG_1920x1080_W=1920 #pixels
DEFINES += IMG_1920x1080_H=1080 #pixels
#DEFINES += IMG_H=540
#DEFINES += IMG_W=960
#DEFINES += CHECKERBOARD_SQUARE_SIZE=93 # en mm
#DEFINES += CHECKERBOARD_SQUARE_SIZE=23 # en mm
#DEFINES += CHECKERBOARD_CORNERS_WIDTH=9
#DEFINES += CHECKERBOARD_CORNERS_HEIGHT=6
#DEFINES += AREA_WIDTH=2900 # en mm
#DEFINES += AREA_HIGH=2775 # en mm
DEFINES += HUE_MIN=0
DEFINES += HUE_MAX=180
DEFINES += SAT_MIN=0
DEFINES += SAT_MAX=255
DEFINES += VAL_MIN=0
DEFINES += VAL_MAX=255
DEFINES += _WINSOCK_DEPRECATED_NO_WARNINGS
#DEFINES += HIGH_LED_ON_BOX=505 #en mm
DEFINES += CLOCK_TIME_POINT=std::chrono::high_resolution_clock::time_point
DEFINES += CLOCK_DURATION=std::chrono::high_resolution_clock::duration
#DEFINES += CLOCK_CONVERT=std::chrono::duration<double>
DEFINES += CLOCK_NOW=std::chrono::high_resolution_clock::now()
DEFINES += POSITION_NOT_FOUND=-9999
#DEFINES += LENGTH_TOOL=400 #mm
#DEFINES += WIDTH_TOOL=40 #mm

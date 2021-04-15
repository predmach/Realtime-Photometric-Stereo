CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(Realtime-Photometric-Stereo)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

## link to .cmake files
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

file(GLOB Sources src/*.cpp)
file(GLOB Headers src/*.h)
file(GLOB Resources *.qrc)
file(GLOB UIs *.ui)

# required packages
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

# Use Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

FIND_PACKAGE(VTK REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(DC1394 REQUIRED)
FIND_PACKAGE(OPENCL REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)

# ignore deprecated OpenCL 1.1 headers warning
ADD_DEFINITIONS(-DCL_USE_DEPRECATED_OPENCL_1_1_APIS)

# add -lGLU to linker options
SET(QT_USE_QTOPENGL TRUE)

# required definitions used when compiling qt code
ADD_DEFINITIONS(${QT_DEFINITIONS})

# add custom definitions
SET(PATH_ASSETS \"${CMAKE_SOURCE_DIR}/assets/\")
SET(PATH_KERNELS \"${CMAKE_SOURCE_DIR}/src/\")
SET(START_LED 3)
SET(IMG_HEIGHT 480)
SET(IMG_WIDTH 640)
CONFIGURE_FILE(src/config.h.in ${CMAKE_SOURCE_DIR}/src/config.h)

# compiling qt gui (qmake) and adding resource files
#QT5_WRAP_CPP(main_headers_moc
#			src/mainwindow.h
#                        src/lights.h
#            src/camera.h
#            src/rs_cam.h
#            src/camerawidget.h
#            src/modelwidget.h
#            src/photometricstereo.h)

#FILE(GLOB SOURCES 	${CMAKE_SOURCE_DIR}/src/*.cpp
#				  	${CMAKE_SOURCE_DIR}/src/*.h
#				  	${CMAKE_SOURCE_DIR}/src/*.cl)

ADD_EXECUTABLE(main ${Sources} ${Headers} ${Resources} ${UIs})

LINK_LIBRARIES(${QT_LIBRARIES})
LINK_LIBRARIES(${VTK_LIBRARIES})
LINK_LIBRARIES(${realsense2_LIBRARY})
TARGET_LINK_LIBRARIES(main ${DC1394_LIBRARIES} ${OPENCL_LIBRARIES} ${OpenCV_LIBS} Qt5::Widgets Qt5::Gui ${VTK_LIBRARIES} ${realsense2_LIBRARY})

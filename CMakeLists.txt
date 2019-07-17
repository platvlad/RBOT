CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

SET(PROJECTNAME RBOT)
PROJECT(${PROJECTNAME})

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(OpenCV_STATIC OFF)


SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOMOC ON)

FILE(GLOB SOURCES src/*.cpp src/*.h src/*.hpp src/*.glsl)

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Qt5Widgets REQUIRED)
FIND_PACKAGE(Qt5OpenGL REQUIRED)
FIND_PACKAGE(ASSIMP REQUIRED)
FIND_PACKAGE( Boost 1.66 COMPONENTS filesystem program_options system REQUIRED )

SET(LIBRARIES 
	Qt5::Widgets 
	Qt5::OpenGL
	${OpenCV_LIBS}
	${OPENGL_LIBRARIES}
	${ASSIMP_LIBRARIES}
	${Boost_LIBRARIES}
	${PROJECT_SOURCE_DIR}/deps/yaml-cpp-release-0.3.0/build/libyaml-cpp.a
)


INCLUDE_DIRECTORIES(
	${OpenCV_INCLUDE_DIRS} 
	${OPENGL_INCLUDE_DIR}
	${ASSIMP_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
	${PROJECT_SOURCE_DIR}/deps/yaml-cpp-release-0.3.0/include/
)


ADD_EXECUTABLE(${PROJECTNAME} ${SOURCES} src/argparsing.h src/argparsing.cpp src/OptProcessor.h src/OptProcessor.cpp src/RBOTTestRunner.h src/RBOTTestRunner.cpp src/io.hpp src/io.cpp src/data.hpp src/data.cpp src/FrameGetter.h src/StrictFrameGetter.h src/StrictFrameGetter.cpp src/SingleIterationFrameGetter.h src/SingleIterationFrameGetter.cpp src/EnhancingFrameGetter.h src/EnhancingFrameGetter.cpp)

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${LIBRARIES})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

################Initialize the Cmake Defaults#################
cmake_minimum_required(VERSION 2.6)

#Name the project
project(PumaClient)

#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the lib and src directories#################

#Set scl base directory
SET(SCL_BASE_DIR ../../)

###(a) Scl controller
SET(SCL_INC_DIR ${SCL_BASE_DIR}src/scl/)
SET(SCL_INC_DIR_BASE ${SCL_BASE_DIR}src/)
SET(TAO_INC_DIR ${SCL_INC_DIR}dynamics/tao/)
ADD_DEFINITIONS(-DTIXML_USE_STL)

###(b) Eigen
SET(EIGEN_INC_DIR ${SCL_BASE_DIR}3rdparty/eigen/)

###(c) Chai3D scenegraph
SET(CHAI_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/chai3d/)
ADD_DEFINITIONS(-D_LINUX -DLINUX)

### (d) sUtil code
SET(SUTIL_INC_DIR ${SCL_BASE_DIR}3rdparty/sUtil/src/)

### (e) scl_tinyxml (parser)
SET(TIXML_INC_DIR ${SCL_BASE_DIR}3rdparty/tinyxml)

#Set OpenCV base directory
SET(OpenCV_DIR ~/opencv-2.4.10/build)

SET(TRACK_DIR ../Kinect)

#Set all the sources required for the library
SET(SRC_BASE_DIR ${SCL_BASE_DIR}/workspace/Puma)

################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(${SCL_INC_DIR} ${SCL_INC_DIR_BASE} ${TAO_INC_DIR} ${EIGEN_INC_DIR} ${CHAI_INC_DIR} ${SUTIL_INC_DIR} ${TIXML_INC_DIR} ${OpenCV_INCLUDE_DIRS} ${TRACK_DIR}) 

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++0x -DGRAPHICS_ON -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x -DGRAPHICS_ON -DW_THREADING_ON -DNDEBUG")

#These are the source files that will be compiled.
#####Simulation
SET(ALL_SRC #${TUTE3_BASE_DIR}/scl_tutorial4_control_gc_op.cpp
            ${SCL_INC_DIR}/graphics/chai/CGraphicsChai.cpp 
            ${SCL_INC_DIR}/graphics/chai/ChaiGlutHandlers.cpp)

#####Experiment
SET(TOP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

SET(SOURCES ${SRC_BASE_DIR}/scl_tutorial4_control_gc_op_simple.cpp 
            ${SRC_BASE_DIR}/PumaClient.cpp
            ${SRC_BASE_DIR}/Robot/RobotCom.cpp
            ${SRC_BASE_DIR}/Robot/unitMsg.cpp
            ${SRC_BASE_DIR}/Robot/cs225.h
)

#add_executable( ${PROJECT_NAME} ${SOURCES} )

INCLUDE_DIRECTORIES(Robot)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(${PROJECT_NAME} ${ALL_SRC} ${SOURCES})

###############SPECIAL CODE TO FIND AND LINK SCL's LIB DIR ######################
find_library( SCL_LIBRARY_DEBUG NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES debug )

find_library( SCL_LIBRARY_RELEASE NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES release )

SET( SCL_LIBRARY debug     ${SCL_LIBRARY_DEBUG}
              optimized ${SCL_LIBRARY_RELEASE} )

target_link_libraries(${PROJECT_NAME} ${SCL_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK CHAI's LIB DIR ######################
find_library( CHAI_LIBRARY_DEBUG NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES debug )

find_library( CHAI_LIBRARY_RELEASE NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES release )

SET( CHAI_LIBRARY debug     ${CHAI_LIBRARY_DEBUG}
              optimized ${CHAI_LIBRARY_RELEASE} )

target_link_libraries(${PROJECT_NAME} ${CHAI_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK OpenCV's LIB DIR ######################
find_package( OpenCV REQUIRED )

find_library( TRACK_LIB NAMES BallTracker
            PATHS ${TRACK_DIR}/build )

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${TRACK_LIB})

###############CODE TO FIND AND LINK REMANING LIBS ######################
target_link_libraries(${PROJECT_NAME} gomp GL GLU GLEW glut ncurses rt dl)




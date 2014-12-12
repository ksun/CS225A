################Initialize the Cmake Defaults#################
cmake_minimum_required(VERSION 2.6)

#Name the project
project(CvPlot)

#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the 3rdParty lib#################

#Set OpenCV base directory
SET(OpenCV_DIR ~/opencv-2.4.10/build)
find_package( OpenCV REQUIRED )

################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS}) 

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++0x -DGRAPHICS_ON -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x -DGRAPHICS_ON -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
SET(SRC_BASE_DIR ..)
 
#These are the source files that will be compiled.
SET(ALL_SRC  ${SRC_BASE_DIR}/plot_sample.cpp)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(plot_sample ${ALL_SRC})
add_library(CvPlot ${SRC_BASE_DIR}/cvplot.cpp)

###############CODE TO FIND AND LINK REMANING LIBS ######################
target_link_libraries( plot_sample ${OpenCV_LIBS} CvPlot)


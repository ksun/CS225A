cmake_minimum_required(VERSION 2.4.6)

# building an example file
add_executable(exampleCode src/exampleCode.cpp )
target_link_libraries(exampleCode OptiTrack )

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# building the optiTrack library
add_library(OptiTrack src/OptiTrack.cpp include/OptiTrack.h)

include_directories("./include")
target_link_libraries(OptiTrack vrpn pthread )


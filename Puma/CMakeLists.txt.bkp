CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT (PumaClient)
SET(TOP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)

############## CMAKE OPTIONS #####################

SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
SET(CMAKE_BUILD_TYPE Release)

############### SOURCE FILES ######################

SET(SOURCES PumaClient.cpp
            Robot/RobotCom.cpp
            Robot/unitMsg.cpp
            Robot/cs225.h
)



add_executable( ${PROJECT_NAME} ${SOURCES} )

INCLUDE_DIRECTORIES(Robot)

############### PROTOBUF DEPENDENDCIES ######################
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} -l<staticlibname>)







PROJECT(RAPP_CALIBRATE_CAMERA)

cmake_minimum_required(VERSION 2.8.3)

find_package(Rapp REQUIRED)
find_package(Boost COMPONENTS system random REQUIRED)
find_package(RappRobots REQUIRED)
include_directories(${RAPP_INCLUDE_DIRS}/rapp)

find_package(OpenCV REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${Rapp_DIR} ${RappRobots_DIR})
include(RappMacros)

find_library(PTHREAD NAMES libpthread.so.0 )

setup_rapp()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -fPIC -static-libstdc++ -mtune=atom -march=atom -m32")
#set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -m64")

create_rapp(
    NAME LearnObject
    SRCS main.cpp
    LIBS ${PTHREAD} ${Boost_LIBRARIES} dl ${OpenCV_LIBS}
 #   [FILES file1, file2, ...]
)

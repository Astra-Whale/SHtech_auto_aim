cmake_minimum_required(VERSION 3.15)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

FILE(GLOB COMM_SRC_LIST *.cpp)

add_library(comm OBJECT ${COMM_SRC_LIST})
target_link_libraries(comm serial ${OpenCV_LIBS})

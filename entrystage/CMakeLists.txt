cmake_minimum_required(VERSION 3.15)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

FILE(GLOB ENTRYSTAGE_SRC_LIST 
    entryStage_submodule.cpp
)

add_library(entrystage OBJECT ${ENTRYSTAGE_SRC_LIST})
target_link_libraries(entrystage ${OpenCV_LIBS})

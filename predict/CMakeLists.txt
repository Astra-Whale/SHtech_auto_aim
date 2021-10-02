cmake_minimum_required(VERSION 3.15)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

FILE(GLOB SENSOR_SRC_LIST predict.cpp)

add_library(predict OBJECT ${SENSOR_SRC_LIST})
target_link_libraries(predict ${OpenCV_LIBS} ${CERES_LIBRARIES})

cmake_minimum_required(VERSION 3.15)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

FILE(GLOB DETECT_SRC_LIST *.cpp)
FILE(GLOB TensorRT_LIBS /usr/lib/aarch64-linux-gnu/libnv*.so)

add_library(detect OBJECT ${DETECT_SRC_LIST})
target_link_libraries(detect ${OpenCV_LIBS} ${TensorRT_LIBS} ${CUDA_LIBRARIES})

cmake_minimum_required(VERSION 3.15)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CUDA_INCLUDE_DIRS})

FILE(GLOB DETECT_SRC_LIST *.cpp)
#按TENSOR RT安装位置更改路径
FILE(GLOB TensorRT_LIBS /usr/lib/aarch64-linux-gnu/libnv*.so)
#如头文件不在默认引用路径中需手动添加 include_directories
#include_directories(/path/to/tensorrt_incs)

add_library(detect OBJECT ${DETECT_SRC_LIST})
target_link_libraries(detect ${OpenCV_LIBS} ${TensorRT_LIBS} ${CUDA_LIBRARIES})

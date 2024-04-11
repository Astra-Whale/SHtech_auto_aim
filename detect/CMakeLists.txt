cmake_minimum_required(VERSION 3.15)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(/usr/local/ncnn/include)
FILE(GLOB DETECT_SRC_LIST *.cpp)

add_library(detect OBJECT ${DETECT_SRC_LIST})
target_link_libraries(detect ${OpenCV_LIBS} /usr/local/ncnn/lib/libncnn.so)

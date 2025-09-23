cmake_minimum_required(VERSION 3.15)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# 只编译新的 SubModule 和相关的预测器文件，移除原有的 predict.cpp
FILE(GLOB PREDICT_SRC_LIST 
    predict_submodule.cpp
    LinearPredictor.cpp
    StaticPredictor.cpp
)

add_library(predict OBJECT ${PREDICT_SRC_LIST})
target_link_libraries(predict ${OpenCV_LIBS})

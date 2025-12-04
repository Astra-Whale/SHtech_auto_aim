cmake_minimum_required(VERSION 3.15)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# # 只编译新的合并后的 LinearPredictorSubModule
# FILE(GLOB PREDICT_SRC_LIST 
#     linearPredictor_submodule.cpp
# )

FILE(GLOB PREDICT_SRC_LIST *.cpp)

add_subdirectory(./tinympc)

add_library(predict OBJECT ${PREDICT_SRC_LIST})
target_link_libraries(predict ${OpenCV_LIBS} tinympcstatic)

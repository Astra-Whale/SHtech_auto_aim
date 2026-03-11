cmake_minimum_required(VERSION 3.15)

# 设置库名称
set(MODULE_NAME mathutils)

# 查找依赖包
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# 收集源文件
file(GLOB_RECURSE SOURCES "*.cpp")

# 创建静态库
add_library(${MODULE_NAME} STATIC ${SOURCES})

# 链接依赖库
target_link_libraries(${MODULE_NAME} ${OpenCV_LIBS})
target_link_libraries(${MODULE_NAME} Eigen3::Eigen)

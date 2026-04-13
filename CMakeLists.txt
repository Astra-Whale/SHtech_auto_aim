CMAKE_MINIMUM_REQUIRED(VERSION 3.12)
project(auto-aim)
SET(exe_name auto-aim)
set(CMAKE_CXX_STANDARD 20)

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif ()

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

set(INFERENCE_BACKEND "AXCL" CACHE STRING "Select inference backend: ONNX, TRT, AXCL")
set_property(CACHE INFERENCE_BACKEND PROPERTY STRINGS ONNX TRT AXCL)

option(ENABLE_FOXGLOVE "Build with Foxglove module and thirdparty SDK" OFF)

# Auto-detect platform for Foxglove SDK
if(ENABLE_FOXGLOVE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(DEFAULT_FOXGLOVE_PLATFORM "arm64")
    else()
        set(DEFAULT_FOXGLOVE_PLATFORM "x64")
    endif()

    set(FOXGLOVE_PLATFORM "${DEFAULT_FOXGLOVE_PLATFORM}" CACHE STRING "Select Foxglove SDK platform: x64, arm64")
    set_property(CACHE FOXGLOVE_PLATFORM PROPERTY STRINGS x64 arm64)
endif()

option(USE_HIKCAM "Build with HikCAM Lib" ON)

set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)

add_compile_options(-Wall -Werror=return-type -Wno-unused-variable)
if (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(-O3 -flto -march=native)
    #add_compile_options(-O3 -flto -march=native -g -fsanitize=address)
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    add_compile_options(-O3 -flto -march=native -g)
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    add_compile_options(-Os -flto -march=native)
else ()
    add_compile_options(-O0 -g)
endif ()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

if(ENABLE_FOXGLOVE)
    add_compile_definitions(ENABLE_FOXGLOVE)

    # Configure Foxglove SDK
    set(foxglove_SOURCE_DIR "${PROJECT_SOURCE_DIR}/thirdparty/foxglove_${FOXGLOVE_PLATFORM}")
    message(STATUS "Using Foxglove SDK from: ${foxglove_SOURCE_DIR}")

    # Add Foxglove SDK include directories
    include_directories(${foxglove_SOURCE_DIR}/include)
    include_directories(${foxglove_SOURCE_DIR}/include/foxglove)

    # Find all Foxglove SDK source files
    file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
        "${foxglove_SOURCE_DIR}/src/*.cpp"
        "${foxglove_SOURCE_DIR}/src/server/*.cpp"
    )
endif()

add_executable(${exe_name} main.cpp)


# 1. 设置编译标志
#target_compile_options(auto-aim PRIVATE -fsanitize=address -fno-omit-frame-pointer)

# 2. 设置链接标志 (关键！这一步告诉链接器去链接 libasan)
#target_link_options(auto-aim PRIVATE -fsanitize=address)

include_directories(common)
include_directories(entrystage)
include_directories(sensor)
include_directories(detect)
include_directories(predict)
include_directories(timedserial)
include_directories(planner)
include_directories(mathutils)

if(ENABLE_FOXGLOVE)
    include_directories(foxglove)
endif()

add_subdirectory(common)
add_subdirectory(entrystage)
add_subdirectory(detect)
add_subdirectory(predict)
add_subdirectory(sensor)
add_subdirectory(timedserial)
add_subdirectory(planner)
add_subdirectory(mathutils)

if(ENABLE_FOXGLOVE)
    add_subdirectory(foxglove)
endif()

target_link_libraries(${exe_name} common)
target_link_libraries(${exe_name} entrystage)
target_link_libraries(${exe_name} detect)
target_link_libraries(${exe_name} predict)
target_link_libraries(${exe_name} sensor)
target_link_libraries(${exe_name} timedserial)
target_link_libraries(${exe_name} planner)
target_link_libraries(${exe_name} mathutils)

if(ENABLE_FOXGLOVE)
    target_link_libraries(${exe_name} foxglove)
endif()

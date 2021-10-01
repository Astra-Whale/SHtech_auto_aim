CMAKE_MINIMUM_REQUIRED(VERSION 3.12)
SET(exe_name auto-aim-test)
SET(MVS_LD_FLAGS "-Wl,-rpath,")
SET(MVS_LD_FLAGS ${MVS_LD_FLAGS}$ENV{MVS_PATH}" -R '"$ENV{MVS_PATH}"'")
set(CMAKE_CXX_STANDARD 17)
project(${exe_name})


SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -std=c++17")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

FILE(GLOB SRC_LIST_CPP ${PROJECT_SOURCE_DIR}/*.cpp
                       ${PROJECT_SOURCE_DIR}/detector/*.cpp 
                       ${PROJECT_SOURCE_DIR}/hikcam_wrapper/*.cpp
                       ${PROJECT_SOURCE_DIR}/comm/*.cpp
                       ${PROJECT_SOURCE_DIR}/Log/*.cpp
                       ${PROJECT_SOURCE_DIR}/timer/*.cpp
                       ${PROJECT_SOURCE_DIR}/cmd_parser/*.cpp
                       /usr/src/tensorrt/samples/common/*.cpp
                       )

FILE(GLOB TENSORRT_LIBS "/usr/lib/aarch64-linux-gnu/*nv*.so")

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/inc)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/detector)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/hikcam_wrapper)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/hikcam_wrapper/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/comm)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Log)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/timer)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/cmd_parser)
INCLUDE_DIRECTORIES(/usr/src/tensorrt/samples/common)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(CUDA REQUIRED)

if(NOT DEFINED ENV{MVS_PATH})
    message(FATAL_ERROR "not defined env: MVS_PATH")
endif()
LINK_DIRECTORIES($ENV{MVS_PATH})

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})

ADD_EXECUTABLE(${exe_name} ${SRC_LIST_CPP})

LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

TARGET_LINK_LIBRARIES(${exe_name} ${TENSORRT_LIBS})
TARGET_LINK_LIBRARIES(${exe_name} ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(${exe_name} ${CUDA_LIBRARIES})
TARGET_LINK_LIBRARIES(${exe_name} ${CMAKE_THREAD_LIBS_INIT})
TARGET_LINK_LIBRARIES(${exe_name} serial)
TARGET_LINK_LIBRARIES(${exe_name} libMvCameraControl.so)
set_target_properties(${exe_name} PROPERTIES LINK_FLAGS ${MVS_LD_FLAGS})

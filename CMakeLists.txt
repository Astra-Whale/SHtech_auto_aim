CMAKE_MINIMUM_REQUIRED(VERSION 3.12)
SET(exe_name auto-aim-test)
SET(MVS_LD_FLAGS "-Wl,-rpath,")
SET(MVS_LD_FLAGS ${MVS_LD_FLAGS}$ENV{MVS_PATH}" -R '"$ENV{MVS_PATH}"'")
project(${exe_name})


SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

FILE(GLOB SRC_LIST_CPP ${PROJECT_SOURCE_DIR}/src/*.cpp
                       ${PROJECT_SOURCE_DIR}/src/detector/*.cpp 
                       ${PROJECT_SOURCE_DIR}/src/tracker/*.cpp 
                       ${PROJECT_SOURCE_DIR}/others/hikcam_wrapper/*.cpp
                       ${PROJECT_SOURCE_DIR}/others/comm/*.cpp
                       ${PROJECT_SOURCE_DIR}/others/Log/*.cpp
                       ${PROJECT_SOURCE_DIR}/others/timer/*.cpp
                       ${PROJECT_SOURCE_DIR}/others/cmd_parser/*.cpp
                       )

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/inc)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/inc/detector)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/inc/tracker)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/others/hikcam_wrapper)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/others/hikcam_wrapper/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/others/yolo/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/others/comm)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/others/Log)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/others/timer)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/others/cmd_parser)

find_package(OpenCV REQUIRED)
find_package (Threads REQUIRED)

if(NOT DEFINED ENV{MVS_PATH})
    message(FATAL_ERROR "not defined env: MVS_PATH")
endif()
LINK_DIRECTORIES($ENV{MVS_PATH})

include_directories(${OpenCV_INCLUDE_DIRS})

LINK_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR}/others/yolo/lib)

ADD_EXECUTABLE(${exe_name} ${SRC_LIST_CPP})
link_directories(${OpenCV_LIBRARY_DIRS})
TARGET_LINK_LIBRARIES(${exe_name} libdarknet.so)
target_link_libraries(${exe_name}  ${OpenCV_LIBS})
target_link_libraries(${exe_name} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${exe_name} libMvCameraControl.so)
set_target_properties(${exe_name} PROPERTIES LINK_FLAGS ${MVS_LD_FLAGS})
target_link_libraries(${exe_name} serial)

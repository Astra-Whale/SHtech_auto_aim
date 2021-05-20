# # project name
# PROJECT(HELLO)
# SET(MVS_LD_FLAGS "-Wl,-rpath,")
# SET(MVS_LD_FLAGS ${MVS_LD_FLAGS}$ENV{MVS_PATH}" -R '"$ENV{MVS_PATH}"'")

# # using C++11
# set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 ")

# # cmake version
# CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# # find OpenCV
# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package (Threads REQUIRED)

# # show the message of OpenCV
# message(STATUS "OpenCV library status:")
# message(STATUS "    version: 	${OpenCV_VERSION}")
# message(STATUS "    headers: 	${OpenCV_INCLUDE_DIRS}")
# message(STATUS "    libraries: 	${OpenCV_LIBS}")

# FILE(GLOB SRC_LIST_CPP ${PROJECT_SOURCE_DIR}/camera_param/*.cpp
#                        ${PROJECT_SOURCE_DIR}/../hikcam_wrapper/*.cpp
#                        )

# # link headers
# INCLUDE_DIRECTORIES({OpenCV_INCLUDE_DIRS})
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/../hikcam_wrapper)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/../hikcam_wrapper/include)

# # 添加源代码文件到SRC_LIST变量中
# AUX_SOURCE_DIRECTORY(. SRC_LIST_CPP)

# # 生成可执行文件
# ADD_EXECUTABLE(hello ${SRC_LIST_CPP})

# # after ADD_EXECUTABLE，为生成文件target添加库
# TARGET_LINK_LIBRARIES(hello ${OpenCV_LIBS})

# if(NOT DEFINED ENV{MVS_PATH})
#     message(FATAL_ERROR "not defined env: MVS_PATH")
# endif()
# LINK_DIRECTORIES($ENV{MVS_PATH})

# include_directories(${OpenCV_INCLUDE_DIRS})



# link_directories(${OpenCV_LIBRARY_DIRS})

# target_link_libraries(hello  ${OpenCV_LIBS})
# target_link_libraries(hello ${CMAKE_THREAD_LIBS_INIT})
# # target_link_libraries(hello libMvCameraControl.so)
# set_target_properties(hello PROPERTIES LINK_FLAGS ${MVS_LD_FLAGS})

CMAKE_MINIMUM_REQUIRED(VERSION 3.12)
SET(exe_name hello)
SET(MVS_LD_FLAGS "-Wl,-rpath,")
SET(MVS_LD_FLAGS ${MVS_LD_FLAGS}$ENV{MVS_PATH}" -R '"$ENV{MVS_PATH}"'")
project(${exe_name})


SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

FILE(GLOB SRC_LIST_CPP ${PROJECT_SOURCE_DIR}/../hikcam_wrapper/*.cpp ${PROJECT_SOURCE_DIR}/*.cpp)


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/../hikcam_wrapper)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/../hikcam_wrapper/include)

find_package(OpenCV REQUIRED)

if(NOT DEFINED ENV{MVS_PATH})
    message(FATAL_ERROR "not defined env: MVS_PATH")
endif()
LINK_DIRECTORIES($ENV{MVS_PATH})

include_directories(${OpenCV_INCLUDE_DIRS})


ADD_EXECUTABLE(${exe_name} ${SRC_LIST_CPP})
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(${exe_name}  ${OpenCV_LIBS})
target_link_libraries(${exe_name} libMvCameraControl.so)
set_target_properties(${exe_name} PROPERTIES LINK_FLAGS ${MVS_LD_FLAGS})

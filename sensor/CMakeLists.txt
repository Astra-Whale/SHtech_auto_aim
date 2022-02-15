CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(video video_src)
aux_source_directory(hikcam hikcam_src)
aux_source_directory(UartIMU uartimu_src)

#Hik Cam Supporting libraries
SET(MVS_LD_FLAGS "-Wl,-rpath,")
SET(MVS_LD_FLAGS ${MVS_LD_FLAGS}$ENV{MVS_PATH}" -R '"$ENV{MVS_PATH}"'")
if(NOT DEFINED ENV{MVS_PATH})
    message(FATAL_ERROR "not defined env: MVS_PATH")
endif()
LINK_DIRECTORIES($ENV{MVS_PATH})

FILE(GLOB MVS_LIBS "$ENV{MVS_PATH}/lib*.so")

ADD_LIBRARY(sensor OBJECT sensor.cpp ${video_src} ${hikcam_src} ${bmi160_src} ${i2c_src} ${uartimu_src})
target_link_libraries(sensor ${OpenCV_LIBS})
target_link_libraries(sensor libMvCameraControl.so ${MVS_LIBS})
set_target_properties(sensor PROPERTIES LINK_FLAGS ${MVS_LD_FLAGS})

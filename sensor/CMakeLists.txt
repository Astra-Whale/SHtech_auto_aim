CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(video video_src)
if(USE_HIKCAM)
    aux_source_directory(hikcam hikcam_src)
    #Hik Cam Supporting libraries
    SET(MVS_LD_FLAGS "-Wl,-rpath,")
    SET(MVS_LD_FLAGS ${MVS_LD_FLAGS}$ENV{MVS_PATH}" -R '"$ENV{MVS_PATH}"'")
    if(NOT DEFINED ENV{MVS_PATH})
        message(FATAL_ERROR "not defined env: MVS_PATH")
    endif()
    LINK_DIRECTORIES($ENV{MVS_PATH})

    FILE(GLOB MVS_LIBS "$ENV{MVS_PATH}/lib*.so")
    # 只编译新的 SubModule，移除原有的 sensor.cpp 和 UartIMU
    ADD_LIBRARY(sensor OBJECT sensor_submodule.cpp ${video_src} ${hikcam_src})

    target_compile_definitions(sensor PRIVATE ENABLE_HIKCAM=1)
    target_link_libraries(sensor ${OpenCV_LIBS})
    target_link_libraries(sensor libMvCameraControl.so ${MVS_LIBS})
    set_target_properties(sensor PROPERTIES LINK_FLAGS ${MVS_LD_FLAGS})
else()

    # 只编译新的 SubModule，移除原有的 sensor.cpp 和 UartIMU
    ADD_LIBRARY(sensor OBJECT sensor_submodule.cpp ${video_src})
    target_link_libraries(sensor ${OpenCV_LIBS})

endif()

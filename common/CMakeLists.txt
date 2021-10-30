CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(log log_src)
aux_source_directory(timer timer_src)
aux_source_directory(cmd_parser cmd_parser_src)

ADD_LIBRARY(common OBJECT ${log_src} ${timer_src} ${cmd_parser_src})
target_link_libraries(common ${OpenCV_LIBS})

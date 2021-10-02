CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Threads REQUIRED)

aux_source_directory(log log_src)
aux_source_directory(timer timer_src)
aux_source_directory(cmd_parser cmd_parser_src)

ADD_LIBRARY(common OBJECT ${log_src} ${timer_src} ${cmd_parser_src})

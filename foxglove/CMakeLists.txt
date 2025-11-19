CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

find_package(Threads REQUIRED)

# 创建cboard库
ADD_LIBRARY(foxglove_server OBJECT foxglove_server.cpp ${uartimu_src})

# 设置C++标准
set_property(TARGET foxglove_server PROPERTY CXX_STANDARD 20)
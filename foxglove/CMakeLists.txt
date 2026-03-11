CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

find_package(Threads REQUIRED)

# 创建foxglove_server库,包含SDK源文件
ADD_LIBRARY(foxglove OBJECT foxglove_server.cpp ${FOXGLOVE_SOURCES})

# 设置C++标准
set_property(TARGET foxglove PROPERTY CXX_STANDARD 20)

# 添加Foxglove SDK头文件路径
target_include_directories(foxglove PRIVATE ${foxglove_SOURCE_DIR}/include)
target_include_directories(foxglove PRIVATE ${foxglove_SOURCE_DIR}/include/foxglove)

# 链接Foxglove静态库
target_link_libraries(foxglove PRIVATE ${foxglove_SOURCE_DIR}/lib/libfoxglove.a)
target_link_libraries(foxglove PRIVATE Threads::Threads)
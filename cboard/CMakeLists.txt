CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

find_package(Threads REQUIRED)

# 收集UartIMU源文件
aux_source_directory(UartIMU uartimu_src)

# 创建cboard库
ADD_LIBRARY(cboard OBJECT cboard_submodule.cpp ${uartimu_src})

# 链接RMCVSerial库用于串口通信
target_link_libraries(cboard RMCVSerial)

# 设置C++标准
set_property(TARGET cboard PROPERTY CXX_STANDARD 17)
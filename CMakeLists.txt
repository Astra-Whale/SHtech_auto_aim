CMAKE_MINIMUM_REQUIRED(VERSION 3.12)
project(auto-aim)
SET(exe_name auto-aim)
set(CMAKE_CXX_STANDARD 20)

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif ()

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

add_compile_options(-Wall -Werror=return-type -Wno-unused-variable)
if (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(-O3 -flto -march=native)
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    add_compile_options(-O3 -flto -march=native -g)
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    add_compile_options(-Os -flto -march=native)
else ()
    add_compile_options(-O0 -g)
endif ()

add_executable(${exe_name} main.cpp)

include_directories(comm)
add_subdirectory(comm)
target_link_libraries(${exe_name} comm)

include_directories(common)
add_subdirectory(common)
target_link_libraries(${exe_name} common)

include_directories(detect)
add_subdirectory(detect)
target_link_libraries(${exe_name} detect)

include_directories(predict)
add_subdirectory(predict)
target_link_libraries(${exe_name} predict)

include_directories(sensor)
add_subdirectory(sensor)
target_link_libraries(${exe_name} sensor)

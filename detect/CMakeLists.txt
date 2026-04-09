cmake_minimum_required(VERSION 3.15)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# 检查是否从父项目传入了选项
if(NOT DEFINED INFERENCE_BACKEND)
    message(FATAL_ERROR "INFERENCE_BACKEND must be defined in the parent project")
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
if(INFERENCE_BACKEND STREQUAL "TRT")
    find_package(CUDA REQUIRED)
    include_directories(${CUDA_INCLUDE_DIRS})
    aux_source_directory(TensorRT trt_src)
    
    add_library(detect OBJECT preprocess_submodule.cpp corner_refine_submodule.cpp detect_submodule.cpp armor_corner_optimizer.cpp ${trt_src})
    
    if(DEFINED ENV{TRT_PATH})
        FILE(GLOB TRT_LIBS "$ENV{TRT_PATH}/lib/libnv*.so")
        target_include_directories(detect PRIVATE $ENV{TRT_PATH}/include)
        target_link_directories(detect PRIVATE $ENV{TRT_PATH}/lib)
    else()
        FILE(GLOB TRT_LIBS "/usr/lib/aarch64-linux-gnu/libnv*.so")
    endif()
    
    target_link_libraries(detect ${OpenCV_LIBS} ${CUDA_LIBRARIES} ${TRT_LIBS})
    target_compile_definitions(detect PRIVATE INFERENCE_BACKEND_TYPE=2)

elseif(INFERENCE_BACKEND STREQUAL "ONNX")
    include_directories(/opt/rocm/include)
    aux_source_directory(ONNX onnx_src)

    add_library(detect OBJECT preprocess_submodule.cpp corner_refine_submodule.cpp detect_submodule.cpp armor_corner_optimizer.cpp ${onnx_src})

    target_link_libraries(detect ${OpenCV_LIBS} migraphx_c)
    target_link_directories(detect PUBLIC /opt/rocm/lib)
    target_compile_definitions(detect PRIVATE INFERENCE_BACKEND_TYPE=1)

elseif(INFERENCE_BACKEND STREQUAL "AXCL")
    include_directories(/soc/include)
    aux_source_directory(AXCL axcl_src)

    add_library(detect OBJECT preprocess_submodule.cpp corner_refine_submodule.cpp detect_submodule.cpp armor_corner_optimizer.cpp ${axcl_src})

    target_link_libraries(detect ${OpenCV_LIBS} ax_interpreter ax_sys ax_ivps ax_engine)
    target_link_directories(detect PUBLIC /soc/lib)
    target_compile_definitions(detect PRIVATE INFERENCE_BACKEND_TYPE=3)

else()
    message(FATAL_ERROR "Invalid inference backend selected")
endif()

cmake_minimum_required(VERSION 3.10)
project(YOLOv11-ONNXRuntime-CPP)

add_definitions(-DEIGEN_DONT_VECTORIZE)
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -march=x86-64")

# camera
set(k4a_DIR "/home/lingao/yolo_cpp_rewrite/Azure-Kinect-Sensor-SDK")
find_package(k4a REQUIRED)

# Path to ONNX Runtime
set(ONNXRUNTIME_DIR "/home/lingao/yolo_cpp_rewrite/onnxruntime-linux-x64-gpu-1.17.0")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filters search segmentation)

# AUBO SDK integration (forced ON for this project workflow)
set(USE_AUBO_SDK ON CACHE BOOL "Enable AUBO SDK RPC/IK integration" FORCE)
set(AUBO_SDK_ROOT "/home/lingao/Desktop/aubo_sdk-0.23.1-Linux_x86_64+71831e1" CACHE PATH "Path to aubo_sdk root")

list(APPEND CMAKE_PREFIX_PATH "${AUBO_SDK_ROOT}")
find_package(aubo_sdk CONFIG QUIET)
if(aubo_sdk_FOUND)
    message(STATUS "AUBO SDK found: ${AUBO_SDK_ROOT}")
else()
    message(FATAL_ERROR "AUBO SDK not found at AUBO_SDK_ROOT=${AUBO_SDK_ROOT}. Please set a valid SDK path.")
endif()

include_directories(include ${ONNXRUNTIME_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(/usr/lib/x86_64-linux-gnu)
link_directories(${ONNXRUNTIME_DIR}/lib)

find_package(TIFF REQUIRED)
find_package(GDAL REQUIRED)
find_library(FFI_LIB ffi REQUIRED)

set(SOURCES
    main.cpp
    kinect_camera.cpp

)

add_executable(depth2obj ${SOURCES})
add_executable(get_fruit_centroid ../data_generation/get_fruit_centroid.cpp kinect_camera.cpp)

target_link_libraries(depth2obj PRIVATE
    k4a::k4a
    onnxruntime
    onnxruntime_providers_cuda
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${GDAL_LIBRARIES}
    tiff
    geotiff
    gdal
    ffi
)
# 为目标get_fruit_centroid链接相同或类似的库
target_link_libraries(get_fruit_centroid PRIVATE
    k4a::k4a
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${GDAL_LIBRARIES}
    tiff
    geotiff
    gdal
    ffi
)

if(USE_AUBO_SDK)
    target_compile_definitions(depth2obj PRIVATE USE_AUBO_SDK)
    target_include_directories(depth2obj PRIVATE ${AUBO_SDK_ROOT}/include)
    target_link_directories(depth2obj PRIVATE ${AUBO_SDK_ROOT}/lib)
    target_link_libraries(depth2obj PRIVATE aubo_sdk)
    set_target_properties(depth2obj PROPERTIES BUILD_RPATH "${AUBO_SDK_ROOT}/lib")
endif()

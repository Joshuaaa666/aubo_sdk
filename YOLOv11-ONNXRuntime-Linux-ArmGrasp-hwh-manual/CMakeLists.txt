cmake_minimum_required(VERSION 3.10)
project(YOLOv11-ONNXRuntime-CPP)

add_definitions(-DEIGEN_DONT_VECTORIZE)
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -mavx2 -mavx512f -mfma -mf16c")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -march=x86-64")

# camera
set(k4a_DIR "/home/lingao/yolo_cpp_rewrite/Azure-Kinect-Sensor-SDK")
find_package(k4a REQUIRED)

# Path to ONNX Runtime
set(ONNXRUNTIME_DIR "/home/lingao/yolo_cpp_rewrite/onnxruntime-linux-x64-gpu-1.17.0")

# Path to OpenCV
find_package(OpenCV REQUIRED)

# find_package(ONNXRUNTIME REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filters search segmentation)

# Include directories
include_directories(include ${ONNXRUNTIME_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# Link directories
link_directories(/usr/lib/x86_64-linux-gnu)
link_directories(${ONNXRUNTIME_DIR}/lib)

find_package(TIFF REQUIRED)
find_package(GDAL REQUIRED)       
find_library(FFI_LIB ffi REQUIRED)

# Source files
set(SOURCES
    main.cpp
    engine.cpp
    algo.cpp
    kinect_camera.cpp
    tcp_comm.cpp
    serial.cpp
)

set_source_files_properties(
  engine.cpp
  PROPERTIES
    COMPILE_FLAGS "-mavx2 -mavx512f -mfma -mf16c"
)

# Executable
add_executable(yolo_inference ${SOURCES})

# Link libraries
# target_link_libraries(yolo_inference ${ONNXRUNTIME_LIBS})
target_link_libraries(yolo_inference PRIVATE
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

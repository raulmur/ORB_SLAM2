cmake_minimum_required(VERSION 3.0)
project(ByteTrack)

set(ONNXRUNTIME_DIR ${PROJECT_SOURCE_DIR}/../Thirdparty/onnxruntime-linux-x64-gpu-1.11.0)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${ONNXRUNTIME_DIR}/include
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(ByteTrack SHARED
    BytekalmanFilter.cpp
    BYTETrack.cpp
    detector.cpp
    lapjv.cpp
    STrack.cpp
    dataType.h
    BytekalmanFilter.h
    BYTETrack.h
    detector.h
    lapjv.h
    STrack.h
)

target_link_libraries(ByteTrack
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
)
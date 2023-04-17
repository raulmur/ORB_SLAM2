cmake_minimum_required(VERSION 3.0)
project(ByteTrack)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(ByteTrack SHARED
    BytekalmanFilter.cpp
    ByteTrack.cpp
    detector.cpp
    lapjv.cpp
    STrack.cpp
)

target_link_libraries(ByteTrack
    ${OpenCV_LIBS}
    Eigen3::Eigen
)
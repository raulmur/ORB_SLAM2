cmake_minimum_required(VERSION 2.8)
project(orb_slam_2)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS proto/orb_node.proto
                                                   proto/orb_vocabulary.proto
                                                   proto/orb_word.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_definitions(-Wall  -O3 -march=native -std=c++11 -DCOMPILEDWITHC11)

cs_add_library(${PROJECT_NAME} ${PROTO_SRCS}
src/ORBVocabulary.cc

src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/RosMonoImageGrabber.cc
)

# Build examples

cs_add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

cs_add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

cs_add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

cs_add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

cs_add_executable(ros_mono
src/ros_mono.cc)
target_link_libraries(ros_mono ${PROJECT_NAME})

cs_install()
cs_export()

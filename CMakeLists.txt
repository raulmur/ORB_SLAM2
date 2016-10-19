cmake_minimum_required(VERSION 3.4)

# Define superbuild
# -----------------
# The superbuild retrieves and builds dependencies before building this project -- thus making the build process easy
# and eliminating the need to install build dependencies.

option(DO_SUPERBUILD "Whether a superbuild should be done" ON)

if (DO_SUPERBUILD)
    project(SUPERBUILD NONE)
    # execute the superbuild (this script will be invoked again without the
    # DO_SUPERBUILD option this time)
    include(cmake_modules/superbuild.cmake)

    # Stop processing this file further
    return()
else()
    project(ORB_SLAM2)
endif()

if(NOT DEFINED DEPENDENCIES_PATH)
    message(FATAL_ERROR "Variable DEPENDENCIES_PATH needs to be defined.")
endif(NOT DEFINED DEPENDENCIES_PATH)



IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
list(APPEND CMAKE_PREFIX_PATH
    "${DEPENDENCIES_PATH}/Install/ep_eigen"
    "${DEPENDENCIES_PATH}/Install/ep_pangolin"
    "${DEPENDENCIES_PATH}/Install/ep_opencv"
)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
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
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_range_for)
target_compile_definitions(${PROJECT_NAME} PUBLIC -DCOMPILEDWITHC11)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2${CMAKE_SHARED_LIBRARY_SUFFIX}
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o${CMAKE_SHARED_LIBRARY_SUFFIX}
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_compile_features(rgbd_tum PRIVATE cxx_range_for)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_compile_features(stereo_kitti PRIVATE cxx_range_for)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_compile_features(mono_tum PRIVATE cxx_range_for)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_compile_features(mono_kitti PRIVATE cxx_range_for)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})


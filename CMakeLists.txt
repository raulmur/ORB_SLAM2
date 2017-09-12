cmake_minimum_required(VERSION 3.1)
project(orb-slam2)

SET(ORBSLAM2_LIB_TYPE SHARED)
option( BUILD_EXAMPLES "Build Examples" ON )
option( ORBSLAM2_STATIC_LIB "Orb-slam lib type" OFF )
option( BUILD_THIRDPARTY_LIB "Build Thirdparty Lib" OFF )

if(ORBSLAM2_STATIC_LIB)
	SET(ORBSLAM2_LIB_TYPE STATIC)
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

IF(WIN32)
    add_definitions("-DNOMINMAX")
    add_definitions("-DWINDOWS")
    add_definitions("-DORBSLAM_EXPORT")
    add_definitions("-DCOMPILEDWITHC11")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -bigobj")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -bigobj")
ENDIF()

find_package(OpenCV REQUIRED)

if("${OpenCV_VERSION}" VERSION_LESS "2.4.3")
	message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
endif()

find_package(Eigen3 REQUIRED)

if("${EIGEN3_VERSION}" VERSION_LESS "3.1.0")
	message(FATAL_ERROR "Eigen3 > 3.1.0 not found.")
endif()
find_package(Pangolin REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

add_library(${PROJECT_NAME} ${ORBSLAM2_LIB_TYPE}
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

target_sources(${PROJECT_NAME} PRIVATE 
    bootstrap_windows.bat
    build_windows.bat
    bootstrap_linux.sh
    build.sh
    build_linux.sh
    build_ros.sh)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    DBoW2
    g2o
)

# Build examples
if(BUILD_EXAMPLES)
    add_executable(rgbd_tum 
        Examples/RGB-D/rgbd_tum.cc)
    target_link_libraries(rgbd_tum ${PROJECT_NAME})

    add_executable(stereo_kitti 
        Examples/Stereo/stereo_kitti.cc)
    target_link_libraries(stereo_kitti ${PROJECT_NAME})

    add_executable(stereo_euroc 
        Examples/Stereo/stereo_euroc.cc)
    target_link_libraries(stereo_euroc ${PROJECT_NAME})

    add_executable(mono_tum
        Examples/Monocular/mono_tum.cc)
    target_link_libraries(mono_tum ${PROJECT_NAME})

    add_executable(mono_kitti
        Examples/Monocular/mono_kitti.cc)
    target_link_libraries(mono_kitti ${PROJECT_NAME})

    add_executable(mono_euroc
        Examples/Monocular/mono_euroc.cc)
    target_link_libraries(mono_euroc ${PROJECT_NAME})
endif()

cmake_minimum_required(VERSION 2.8)
project(orb-slam2)

SET(ORBSLAM2_LIB_TYPE SHARED)
option( BUILD_EXAMPLES "Build Examples" ON )
option( ORBSLAM2_STATIC_LIB "Orb-slam lib type" OFF )
option( BUILD_THIRDPARTY_LIB "Build Thirdparty Lib" OFF )

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

if(ORBSLAM2_STATIC_LIB)
	SET(ORBSLAM2_LIB_TYPE STATIC)
endif()
	
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
#else()
#   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

IF(WIN32)
	add_definitions("-DNOMINMAX")
	add_definitions("-DWINDOWS")
	add_definitions("-DORBSLAM_EXPORT")
	add_definitions(-DCOMPILEDWITHC11)
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
if(BUILD_THIRDPARTY_LIB)
add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)
endif()
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(ORB_SLAM_G2O_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so)
set(ORB_SLAM_DBoW2_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)

IF(WIN32)
set(ORB_SLAM_G2O_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/bin/${CMAKE_BUILD_TYPE}/g2o.lib)
set(ORB_SLAM_DBoW2_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/${CMAKE_BUILD_TYPE}/DBoW2.lib)
set(ORM_THIRD_PARTY_LIB ${ORB_SLAM_G2O_LIBRARIES} ${ORB_SLAM_DBoW2_LIBRARIES})
ENDIF()

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

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_SLAM_G2O_LIBRARIES}
${ORB_SLAM_DBoW2_LIBRARIES}
)

# Build examples
if(BUILD_EXAMPLES)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME} ${ORM_THIRD_PARTY_LIB})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME} ${ORM_THIRD_PARTY_LIB})

add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME} ${ORM_THIRD_PARTY_LIB})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME} ${ORM_THIRD_PARTY_LIB})

add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME} ${ORM_THIRD_PARTY_LIB})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME} ${ORM_THIRD_PARTY_LIB})
endif()

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   ADD_DEFINITIONS(-DCOMPILEDWITHC11)
   MESSAGE(STATUS "Using flag -std=c++11.")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   ADD_DEFINITIONS(-DCOMPILEDWITHC0X)
   MESSAGE(STATUS "Using flag -std=c++0x.")
ELSE()
   MESSAGE(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(OpenCV 3.0 QUIET)
IF(NOT OpenCV_FOUND)
   FIND_PACKAGE(OpenCV 2.4.3 QUIET)
   IF(NOT OpenCV_FOUND)
      MESSAGE(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   ENDIF()
ENDIF()

FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)

INCLUDE_DIRECTORIES(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

ADD_LIBRARY(${PROJECT_NAME} SHARED
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

IF(APPLE)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.dylib
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.dylib
  )
ELSE(APPLE)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  )
ENDIF(APPLE)
  

# Build examples

# SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
#
# ADD_EXECUTABLE(rgbd_tum
# Examples/RGB-D/rgbd_tum.cc)
# TARGET_LINK_LIBRARIES(rgbd_tum ${PROJECT_NAME})
#
# SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)
#
# ADD_EXECUTABLE(stereo_kitti
# Examples/Stereo/stereo_kitti.cc)
# TARGET_LINK_LIBRARIES(stereo_kitti ${PROJECT_NAME})
#
# ADD_EXECUTABLE(stereo_euroc
# Examples/Stereo/stereo_euroc.cc)
# TARGET_LINK_LIBRARIES(stereo_euroc ${PROJECT_NAME})


SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

# ADD_EXECUTABLE(mono_tum
# Examples/Monocular/mono_tum.cc)
# TARGET_LINK_LIBRARIES(mono_tum ${PROJECT_NAME})
#
# ADD_EXECUTABLE(mono_kitti
# Examples/Monocular/mono_kitti.cc)
# TARGET_LINK_LIBRARIES(mono_kitti ${PROJECT_NAME})
#
# ADD_EXECUTABLE(mono_euroc
# Examples/Monocular/mono_euroc.cc)
# TARGET_LINK_LIBRARIES(mono_euroc ${PROJECT_NAME})

ADD_EXECUTABLE(mono_main
Examples/Monocular/mono_main.cc)
TARGET_LINK_LIBRARIES(mono_main ${PROJECT_NAME})

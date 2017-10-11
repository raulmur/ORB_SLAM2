cmake_minimum_required(VERSION 2.8.3)

project(orb_slam2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  message_filters
  cv_bridge
)

find_package(OpenCV  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

set(ORB_SLAM2_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../../..)
LIST(APPEND CMAKE_MODULE_PATH ${ORB_SLAM2_SOURCE_DIR}/cmake_modules)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")


set(orb_slam2_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include    # orb_slam2/System.h
  ${ORB_SLAM2_SOURCE_DIR}/include  # System.h
  ${ORB_SLAM2_SOURCE_DIR}          # Thirparty
)

set(orb_slam2_LIBRARIES
  ${ORB_SLAM2_SOURCE_DIR}/lib/libORB_SLAM2.so
  ${ORB_SLAM2_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${ORB_SLAM2_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
)

add_library(libORB_SLAM2 SHARED IMPORTED)
set_property(TARGET libORB_SLAM2 PROPERTY IMPORTED_LOCATION ${ORB_SLAM2_SOURCE_DIR}/lib/libORB_SLAM2.so)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${orb_slam2_INCLUDE_DIRS}
  LIBRARIES libORB_SLAM2 ${orb_slam2_LIBRARIES}
  DEPENDS OpenCV Pangolin Eigen3
)


###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${orb_slam2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

set(LIBRARIES
  ${catkin_LIBRARIES}
  ${orb_slam2_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${Pangolin_LIBRARIES}
)


# Node for monocular camera
add_executable(mono src/ros_mono.cc)
target_link_libraries(mono ${LIBRARIES})


# Node for RGB-D camera
add_executable(rgbd src/ros_rgbd.cc )
target_link_libraries(rgbd ${LIBRARIES})

# Node for stereo camera
add_executable(stereo src/ros_stereo.cc)
target_link_libraries(stereo ${LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS mono rgbd stereo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ${ORB_SLAM2_SOURCE_DIR}/include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


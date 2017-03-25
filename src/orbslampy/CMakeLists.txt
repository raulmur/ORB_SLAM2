cmake_minimum_required(VERSION 2.8)

project( _orbslampy CXX )

get_filename_component( CMAKE_LOCAL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/cmake ABSOLUTE )
get_filename_component( SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE )
get_filename_component( INC_ROOT ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE )

# list( APPEND CMAKE_MODULE_PATH ${CMAKE_LOCAL_ROOT}/modules )

find_package( OpenCV REQUIRED )
find_package( Eigen3 3.1.0 REQUIRED )
find_package( Pangolin REQUIRED )
find_package( PythonLibs REQUIRED )

# orbslam subdir
# add_subdirectory( ORB_SLAM2 )
# find_package( ORBSLAM2 REQUIRED )

#message( "ORBSLAM2_INCLUDE_DIRS: ${ORBSLAM2_INCLUDE_DIRS}" )
#message( "ORBSLAM2_LIBS ${ORBSLAM2_LIBS}" )

set( ${PROJECT_NAME}_SRCS
    ${SRC_ROOT}/src/support_functions.cc
    ${SRC_ROOT}/src/orbslammer.cc
    ${SRC_ROOT}/src/orbslampy_wrap.cxx
)

set( ${PROJECT_NAME}_PRIVATE_HEADERS
    ${INC_ROOT}/include/${PROJECT_NAME}/support_functions.h
)

set( ${PROJECT_NAME}_PUBLIC_HEADERS
    ${INC_ROOT}/include/${PROJECT_NAME}/orbslammer.h
)

include_directories(
    # include directory for this project
    ${INC_ROOT}/include
    # opencv headers
    ${PYTHON_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    # ${ORBSLAM2_INCLUDE_DIRS}
    ${GLOBAL_INCLUDE_ROOT}
)

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
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
add_definitions("-w")

add_library( ${PROJECT_NAME} SHARED
    ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_PRIVATE_HEADERS} ${${PROJECT_NAME}_PUBLIC_HEADERS}
)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

target_link_libraries( ${PROJECT_NAME}
    ${PYTHON_LIBRARY}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ORB_SLAM2
)

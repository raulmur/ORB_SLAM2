cmake_minimum_required(VERSION 2.8)

project( _orbslampy CXX )

get_filename_component( CMAKE_LOCAL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/cmake ABSOLUTE )
get_filename_component( SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE )
get_filename_component( INC_ROOT ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE )

# list( APPEND CMAKE_MODULE_PATH ${CMAKE_LOCAL_ROOT}/modules )

find_package( OpenCV REQUIRED opencv_core)
find_package( Eigen3 3.1.0 REQUIRED )
find_package( Pangolin REQUIRED )
find_package( PythonLibs REQUIRED )
find_package(SWIG REQUIRED)

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


set(CMAKE_SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/swig/")
include(${SWIG_USE_FILE})

set(PYTHON_VERSION "2"
    CACHE STRING "Python version used on tests.")
set_property(CACHE PYTHON_VERSION PROPERTY STRINGS "3" "2")

if(PYTHON_VERSION EQUAL 2)
    find_package(PythonLibs 2)
else()
    find_package(PythonLibs 3)
endif()

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

get_property(_include_directories DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
foreach(_include_directory IN LISTS _include_directories)
    list(APPEND CMAKE_SWIG_FLAGS -I${_include_directory})
endforeach()

macro( SWIG_BUILD name SRCS PRIV_HDRS PUB_HDRS)
    execute_process(COMMAND "${SWIG_EXECUTABLE}" "-python" "-c++" "-debug-classes"
        ${CMAKE_SWIG_FLAGS} "-MM" "${CMAKE_CURRENT_SOURCE_DIR}/swig/${name}.i"
        OUTPUT_VARIABLE ${name}_DEPENDENCIES)

    # Remove the first line
    string(REGEX REPLACE "^.+: +\\\\\n +" ""
        ${name}_DEPENDENCIES "${${name}_DEPENDENCIES}")
    # Clean the end of each line
    string(REGEX REPLACE " +(\\\\)?\n" "\n"
        ${name}_DEPENDENCIES "${${name}_DEPENDENCIES}")
    # Clean beginning of each line
    string(REGEX REPLACE "\n +" "\n"
        ${name}_DEPENDENCIES "${${name}_DEPENDENCIES}")
    # clean paths
    string(REGEX REPLACE "\\\\\\\\" "/"
        ${name}_DEPENDENCIES "${${name}_DEPENDENCIES}")
    string(REGEX REPLACE "\n" ";"
        ${name}_DEPENDENCIES "${${name}_DEPENDENCIES}")

    set(SWIG_MODULE_${name}_EXTRA_DEPS ${${name}_DEPENDENCIES})
    set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/swig/${name}.i PROPERTIES CPLUSPLUS ON)
    swig_add_module(${name} python ${CMAKE_CURRENT_SOURCE_DIR}/swig/${name}.i ${SRCS} ${PRIV_HDRS} ${PUB_HDRS})
    swig_link_libraries(${name} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Pangolin_LIBRARIES} ORB_SLAM2)
endmacro(SWIG_BUILD)

#file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/swig/orbslampy_wrap.cxx
#      DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/src/)

# orbslam subdir
# add_subdirectory( ORB_SLAM2 )
# find_package( ORBSLAM2 REQUIRED )

#message( "ORBSLAM2_INCLUDE_DIRS: ${ORBSLAM2_INCLUDE_DIRS}" )
#message( "ORBSLAM2_LIBS ${ORBSLAM2_LIBS}" )

set( ${PROJECT_NAME}_SRCS
    ${SRC_ROOT}/src/support_functions.cc
    ${SRC_ROOT}/src/orbslammer.cc
    # SRC_ROOT}/src/orbslampy_wrap.cxx
)

set( ${PROJECT_NAME}_PRIVATE_HEADERS
    ${INC_ROOT}/include/${PROJECT_NAME}/support_functions.h
)

set( ${PROJECT_NAME}_PUBLIC_HEADERS
    ${INC_ROOT}/include/${PROJECT_NAME}/orbslammer.h
)

set(CMAKE_SWIG_OUTDIR ${LIBRARY_OUTPUT_PATH})

swig_build(orbslampy "${${PROJECT_NAME}_SRCS}" "${${PROJECT_NAME}_PRIVATE_HEADERS}" "${${PROJECT_NAME}_PUBLIC_HEADERS}")

file(COPY amqpslampy.py DESTINATION ${LIBRARY_OUTPUT_PATH}/.. )

# add_library( ${PROJECT_NAME} SHARED
#    ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_PRIVATE_HEADERS} ${${PROJECT_NAME}_PUBLIC_HEADERS}
#)
#set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

#target_link_libraries( ${PROJECT_NAME}
#    ${PYTHON_LIBRARY}
#    ${OpenCV_LIBS}
#    ${EIGEN3_LIBS}
#    ${Pangolin_LIBRARIES}
#    ORB_SLAM2
#)

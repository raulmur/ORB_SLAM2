cmake_minimum_required(VERSION 3.1)
project(PangolinBuild)

include(ExternalProject)
set(COMMON_CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
                      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
                      -DCMAKE_LINKER=${CMAKE_LINKER}
)

find_package(GLEW REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)

set(PANGOLIN_CMAKE_ARGS ${COMMON_CMAKE_ARGS} 
    -DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}
    -DGLEW_INCLUDE_DIR=${GLEW_INCLUDE_DIR}
    -DGLEW_LIBRARY=${GLEW_LIBRARY}
    -DPYTHON_LIBRARY=${PYTHON_LIBRARY}
    -DPYTHON_INCLUDE_DIR=${PYTHON_INCLUDE_DIR}
    -DBUILD_SHARED_LIBS=True
    -DCPP11_NO_BOOST=True
    -DBUILD_PANGOLIN_VIDEO=True
    -DBUILD_PANGOLIN_GUI=True
    -DBUILD_EXAMPLES=False
    -DBUILD_EXTERN_GLEW=False
    -DBUILD_EXTERN_LIBJPEG=False
    -DBUILD_EXTERN_LIBPNG=False)

ExternalProject_Add(pangolin
  URL https://github.com/stevenlovegrove/Pangolin/archive/v0.3.tar.gz
  URL_HASH SHA256=8881c1102671ea99c241b273474ca7a5b3377fbde9c7d376253424c36d8ed678
  CMAKE_ARGS ${PANGOLIN_CMAKE_ARGS}
  BINARY_DIR ${CMAKE_BINARY_DIR}/Pangolin-build)

if(WIN32)
    ExternalProject_Add_Step(pangolin pangolinExportInstall 
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/Pangolin-prefix/src/Pangolin-build/src/include/pangolin/pangolin_export.h"
                "${CMAKE_INSTALL_PREFIX}/include/pangolin/pangolin_export.h"
        COMMENT "Install pangolin_export"
        DEPENDEES install)
endif()

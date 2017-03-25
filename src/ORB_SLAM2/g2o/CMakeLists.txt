CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_LEGACY_CYGWIN_WIN32 0)

PROJECT(g2o)

SET(g2o_C_FLAGS)
SET(g2o_CXX_FLAGS)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

SET (G2O_LIB_TYPE SHARED)

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# Set the output directory for the build executables and libraries
#IF(WIN32)
#  SET(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
#ELSE(WIN32)
#  SET(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
#ENDIF(WIN32)
#SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
#SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
#SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${g2o_RUNTIME_OUTPUT_DIRECTORY})

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
LIST(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(UNIX)

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
FIND_PACKAGE(OpenMP)
SET(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
IF(OPENMP_FOUND AND G2O_USE_OPENMP)
  SET (G2O_OPENMP 1)
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF(OPENMP_FOUND AND G2O_USE_OPENMP)

# Compiler specific options for gcc
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")

# activate warnings !!!
SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Find Eigen3
SET(EIGEN3_INCLUDE_DIR ${G2O_EIGEN3_INCLUDE})
FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
IF(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
ELSE(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
ENDIF(EIGEN3_FOUND)

# Generate config.h
SET(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(config.h.in ${g2o_SOURCE_DIR}/include/g2o/config.h)

# Set up the top-level include directories
include_directories (
    include
    ${g2o_SOURCE_DIR}/core
    ${g2o_SOURCE_DIR}/types
    ${g2o_SOURCE_DIR}/stuff
    ${G2O_EIGEN3_INCLUDE}
)

set( ${PROJECT_NAME}_TYPES_SOURCES
    src/types/types_sba.cpp
    src/types/types_six_dof_expmap.cpp
    src/types/types_seven_dof_expmap.cpp
)
set( ${PROJECT_NAME}_TYPES_HEADERS
    include/g2o/types/types_sba.h
    include/g2o/types/types_six_dof_expmap.h
    include/g2o/types/types_seven_dof_expmap.h
    include/g2o/types/se3quat.h
    include/g2o/types/se3_ops.h
    include/g2o/types/se3_ops.hpp
)
set( ${PROJECT_NAME}_CORE_SOURCES
    src/core/hyper_graph_action.cpp
    src/core/hyper_graph.cpp
    src/core/marginal_covariance_cholesky.cpp
    src/core/matrix_structure.cpp
    src/core/batch_stats.cpp
    src/core/parameter.cpp
    src/core/cache.cpp
    src/core/optimizable_graph.cpp
    src/core/solver.cpp
    src/core/optimization_algorithm_factory.cpp
    src/core/estimate_propagator.cpp
    src/core/factory.cpp
    src/core/sparse_optimizer.cpp
    src/core/hyper_dijkstra.cpp
    src/core/parameter_container.cpp
    src/core/optimization_algorithm.cpp
    src/core/optimization_algorithm_with_hessian.cpp
    src/core/optimization_algorithm_levenberg.cpp
    src/core/jacobian_workspace.cpp
    src/core/robust_kernel.cpp
    src/core/robust_kernel_factory.cpp
    src/core/robust_kernel_impl.cpp
)
set( ${PROJECT_NAME}_CORE_HEADERS
    include/g2o/core/base_edge.h
    include/g2o/core/base_binary_edge.h
    include/g2o/core/base_binary_edge.hpp
    include/g2o/core/hyper_graph_action.h
    include/g2o/core/base_multi_edge.h
    include/g2o/core/base_multi_edge.hpp
    include/g2o/core/hyper_graph.h
    include/g2o/core/base_unary_edge.h
    include/g2o/core/linear_solver.h
    include/g2o/core/base_unary_edge.hpp
    include/g2o/core/base_vertex.h
    include/g2o/core/marginal_covariance_cholesky.h
    include/g2o/core/base_vertex.hpp
    include/g2o/core/matrix_structure.h
    include/g2o/core/batch_stats.h
    include/g2o/core/openmp_mutex.h
    include/g2o/core/block_solver.h
    include/g2o/core/block_solver.hpp
    include/g2o/core/parameter.h
    include/g2o/core/cache.h
    include/g2o/core/optimizable_graph.h
    include/g2o/core/solver.h
    include/g2o/core/creators.h
    include/g2o/core/optimization_algorithm_factory.h
    include/g2o/core/estimate_propagator.h
    include/g2o/core/optimization_algorithm_property.h
    include/g2o/core/factory.h
    include/g2o/core/sparse_block_matrix.h
    include/g2o/core/sparse_block_matrix.hpp
    include/g2o/core/sparse_optimizer.h
    include/g2o/core/hyper_dijkstra.h
    include/g2o/core/parameter_container.h
    include/g2o/core/optimization_algorithm.h
    include/g2o/core/optimization_algorithm_with_hessian.h
    include/g2o/core/optimization_algorithm_levenberg.h
    include/g2o/core/jacobian_workspace.h
    include/g2o/core/robust_kernel.h
    include/g2o/core/robust_kernel_factory.h
    include/g2o/core/robust_kernel_impl.h
)
set( ${PROJECT_NAME}_STUFF_SOURCES
    src/stuff/timeutil.cpp
    src/stuff/os_specific.c
    src/stuff/string_tools.cpp
    src/stuff/property.cpp
)
set( ${PROJECT_NAME}_STUFF_HEADERS
    include/g2o/stuff/string_tools.h
    include/g2o/stuff/color_macros.h
    include/g2o/stuff/macros.h
    include/g2o/stuff/misc.h
    include/g2o/stuff/timeutil.h
    include/g2o/stuff/os_specific.h
    include/g2o/stuff/property.h
)
set( ${PROJECT_NAME}_SOLVERS_SOURCES

)
set( ${PROJECT_NAME}_SOLVERS_HEADERS
    include/g2o/solvers/linear_solver_dense.h
    include/g2o/solvers/linear_solver_eigen.h
)

# Include the subdirectories
add_library (${PROJECT_NAME} ${G2O_LIB_TYPE}
    ${${PROJECT_NAME}_TYPES_SOURCES} ${${PROJECT_NAME}_TYPES_HEADERS}
    ${${PROJECT_NAME}_CORE_SOURCES} ${${PROJECT_NAME}_TYPES_HEADERS}
    ${${PROJECT_NAME}_STUFF_SOURCES} ${${PROJECT_NAME}_STUFF_HEADERS}
    ${${PROJECT_NAME}_SOLVERS_SOURCES} ${${PROJECT_NAME}_SOLVERS_HEADERS}
)

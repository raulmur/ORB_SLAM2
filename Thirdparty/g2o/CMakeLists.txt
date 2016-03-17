CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_LEGACY_CYGWIN_WIN32 0)

PROJECT(g2o)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Detect OS and define macros appropriately
IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(UNIX)

# For building the CHOLMOD / CSPARSE solvers
FIND_PACKAGE(BLAS REQUIRED)
FIND_PACKAGE(LAPACK REQUIRED)

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

cs_add_library(g2o
#types
include/g2o/types/types_sba.cpp
include/g2o/types/types_six_dof_expmap.cpp
include/g2o/types/types_seven_dof_expmap.cpp
include/g2o/core/hyper_graph_action.cpp
include/g2o/core/hyper_graph.cpp
include/g2o/core/marginal_covariance_cholesky.cpp
include/g2o/core/matrix_structure.cpp
include/g2o/core/batch_stats.cpp             
include/g2o/core/parameter.cpp               
include/g2o/core/cache.cpp                   
include/g2o/core/optimizable_graph.cpp       
include/g2o/core/solver.cpp                  
include/g2o/core/optimization_algorithm_factory.cpp
include/g2o/core/estimate_propagator.cpp     
include/g2o/core/factory.cpp                 
include/g2o/core/sparse_optimizer.cpp  
include/g2o/core/hyper_dijkstra.cpp 
include/g2o/core/parameter_container.cpp     
include/g2o/core/optimization_algorithm.cpp 
include/g2o/core/optimization_algorithm_with_hessian.cpp 
include/g2o/core/optimization_algorithm_levenberg.cpp 
include/g2o/core/jacobian_workspace.cpp 
include/g2o/core/robust_kernel.cpp 
include/g2o/core/robust_kernel_factory.cpp
include/g2o/core/robust_kernel_impl.cpp 
include/g2o/stuff/timeutil.cpp
include/g2o/stuff/os_specific.c    
include/g2o/stuff/string_tools.cpp
include/g2o/stuff/property.cpp       
)

cs_install()
cs_export()


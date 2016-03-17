cmake_minimum_required(VERSION 2.8)
project(dbow2)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(SRCS_DBOW2
  src/BowVector.cpp
  src/FORB.cpp      
  src/FeatureVector.cpp
  src/ScoringObject.cpp)
set(SRCS_DUTILS
  src/Random.cpp
  src/Timestamp.cpp)


cs_add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})

cs_install()
cs_export()

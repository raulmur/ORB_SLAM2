cmake_minimum_required(VERSION 2.8)
project(DBoW2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(HDRS_DBOW2
  include/DBoW2/BowVector.h
  include/DBoW2/FORB.h
  include/DBoW2/FClass.h
  include/DBoW2/FeatureVector.h
  include/DBoW2/ScoringObject.h
  include/DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  src/DBoW2/BowVector.cpp
  src/DBoW2/FORB.cpp
  src/DBoW2/FeatureVector.cpp
  src/DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  include/DBoW2/DUtils/Random.h
  include/DBoW2/DUtils/Timestamp.h)
set(SRCS_DUTILS
  src/DUtils/Random.cpp
  src/DUtils/Timestamp.cpp)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

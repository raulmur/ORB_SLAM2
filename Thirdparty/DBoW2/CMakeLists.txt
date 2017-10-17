cmake_minimum_required(VERSION 3.8)
project(DBoW2)

SET (DBOW2_LIB_TYPE STATIC)

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 ${DBOW2_LIB_TYPE} ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/DBoW2/ DESTINATION include/orbslam2/Thirdparty/DBoW2/DBoW2)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/DUtils/ DESTINATION include/orbslam2/Thirdparty/DBoW2/DUtils)

install(TARGETS DBoW2 EXPORT orb-slam2Config    
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
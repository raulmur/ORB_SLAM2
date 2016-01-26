cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME dbow2)
project(${PROJECTNAME})

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# set(HDRS_DBOW2
  # DBoW2/BowVector.h
  # DBoW2/FORB.h 
  # DBoW2/FClass.h       
  # DBoW2/FeatureVector.h
  # DBoW2/ScoringObject.h   
  # DBoW2/TemplatedVocabulary.h)
# set(SRCS_DBOW2
  # DBoW2/BowVector.cpp
  # DBoW2/FORB.cpp      
  # DBoW2/FeatureVector.cpp
  # DBoW2/ScoringObject.cpp)

# set(HDRS_DUTILS
  # DUtils/Random.h
  # DUtils/Timestamp.h)
# set(SRCS_DUTILS
  # DUtils/Random.cpp
  # DUtils/Timestamp.cpp)

file(GLOB_RECURSE  headers "dbow2/*.h") #_RECURSE
file(GLOB_RECURSE  sources "dbow2/*.cpp") #_RECURSE
set(PROJECT_HEADERS ${headers})
set(PROJECT_SOURCES ${sources})
#message("Headers: ${PROJECT_HEADERS}")


set(dbow2_INCLUDE_DIR ${dbow2_SOURCE_DIR}/dbow2 CACHE PATH "dbow2 include directory" FORCE)

IF(WIN32)
	add_definitions(-DNOMINMAX)
	add_definitions(-DWINDOWS)
ENDIF(WIN32)

# Set the output directory for the build executables and libraries
SET(dbow2_RUNTIME_OUTPUT_DIRECTORY ${dbow2_SOURCE_DIR}/bin CACHE PATH "Target for the binaries")
IF(WIN32)
  SET(dbow2_LIBRARY_OUTPUT_DIRECTORY ${dbow2_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
ELSE(WIN32)
  SET(dbow2_LIBRARY_OUTPUT_DIRECTORY ${dbow2_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
ENDIF(WIN32)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${dbow2_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${dbow2_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dbow2_RUNTIME_OUTPUT_DIRECTORY})

  
##-------------------------------------------------
## Detect OpenCV:
##-------------------------------------------------
message("\n")
message("Searching for OpenCV:")
set(OpenCV_STATIC OFF)
# set(ENV_OPENCV_DIR $ENV{OPENCV_DIR})
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
	message("OpenCV successfully added!")
else()
	message(FATAL_ERROR "OpenCV was not found!")
endif()

# postfix, based on type
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")


# work out the postfix; required where we use OUTPUT_NAME
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(EXE_POSTFIX)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  SET(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  SET(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)


include_directories(
	${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECTNAME} SHARED 
	${PROJECT_HEADERS} 
	${PROJECT_SOURCES} 
)

target_link_libraries(${PROJECTNAME} 
	${OpenCV_LIBS}
)


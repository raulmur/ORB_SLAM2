# - Try to find pthread
#
#  pthread_FOUND - system has pthread
#  pthread_INCLUDE_DIRS - the pthread include directories
#  pthread_LIBRARIES - link these to use pthread

FIND_PATH(
  pthread_INCLUDE_DIRS
  NAMES pthread.h
  PATHS
    c:/dev/sysroot32/usr/include
    ${CMAKE_SOURCE_DIR}/../pthread/include
    /usr/include/
    /usr/local/include
    /opt/local/include
)

FIND_LIBRARY(
  pthread_LIBRARIES
  NAMES pthreadVSE2 pthread
  PATHS
    c:/dev/sysroot32/usr/lib
    ${CMAKE_SOURCE_DIR}/../pthread/lib
    /usr/lib
    /usr/local/lib
    /opt/local/lib
)

IF(pthread_INCLUDE_DIRS AND pthread_LIBRARIES)
  SET(pthread_FOUND TRUE)
ENDIF(pthread_INCLUDE_DIRS AND pthread_LIBRARIES)

IF(pthread_FOUND)
   IF(NOT pthread_FIND_QUIETLY)
      MESSAGE(STATUS "Found pthread: ${pthread_LIBRARIES}")
   ENDIF(NOT pthread_FIND_QUIETLY)
ELSE(pthread_FOUND)
message(STATUS "pthread NOT found")
   IF(pthread_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find pthread")
   ENDIF(pthread_FIND_REQUIRED)
ENDIF(pthread_FOUND)

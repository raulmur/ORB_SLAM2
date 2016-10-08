# - Try to find uvc
#
#  uvc_FOUND - system has libuvc
#  uvc_INCLUDE_DIRS - the libuvc include directories
#  uvc_LIBRARIES - link these to use libuvc

FIND_PATH(
  uvc_INCLUDE_DIRS
  NAMES libuvc/libuvc.h
  PATHS 
    ${CMAKE_SOURCE_DIR}/..
    /usr/include/
    /usr/local/include
    /opt/local/include
)

FIND_LIBRARY(
  uvc_LIBRARIES
  NAMES uvc
  PATHS
    ${CMAKE_SOURCE_DIR}/../uvc/build
    /usr/lib
    /usr/local/lib
    /opt/local/lib
) 

IF (uvc_INCLUDE_DIRS AND uvc_LIBRARIES)
   SET(uvc_FOUND TRUE)
ENDIF (uvc_INCLUDE_DIRS AND uvc_LIBRARIES)

IF (uvc_FOUND)
   IF (NOT uvc_FIND_QUIETLY)
      MESSAGE(STATUS "Found uvc: ${uvc_LIBRARIES}")
   ENDIF (NOT uvc_FIND_QUIETLY)
ELSE (uvc_FOUND)
   IF (uvc_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find uvc")
   ENDIF (uvc_FIND_REQUIRED)
ENDIF (uvc_FOUND)

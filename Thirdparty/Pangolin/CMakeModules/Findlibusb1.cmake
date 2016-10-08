# - Try to find libusb1
#
#  libusb1_FOUND - system has libusb1
#  libusb1_INCLUDE_DIRS - the libusb1 include directories
#  libusb1_LIBRARIES - link these to use libusb1

FIND_PATH(
  libusb1_INCLUDE_DIRS
  NAMES libusb-1.0/libusb.h
  PATHS
    c:/dev/sysroot32/usr/include
    ${CMAKE_SOURCE_DIR}/../libusb1/include
    /usr/include/
    /usr/local/include
    /opt/local/include
)

FIND_LIBRARY(
  libusb1_LIBRARIES
  NAMES libusb-1.0
  PATHS
    c:/dev/sysroot32/usr/lib
    ${CMAKE_SOURCE_DIR}/../libusb1/lib
    /usr/lib
    /usr/local/lib
    /opt/local/lib
)

IF(libusb1_INCLUDE_DIRS AND libusb1_LIBRARIES)
  SET(libusb1_FOUND TRUE)
ENDIF(libusb1_INCLUDE_DIRS AND libusb1_LIBRARIES)

IF(libusb1_FOUND)
   IF(NOT libusb1_FIND_QUIETLY)
      MESSAGE(STATUS "Found libusb1: ${libusb1_LIBRARIES}")
   ENDIF(NOT libusb1_FIND_QUIETLY)
ELSE(libusb1_FOUND)
message(STATUS "libusb1 NOT found")
   IF(libusb1_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find libusb1")
   ENDIF(libusb1_FIND_REQUIRED)
ENDIF(libusb1_FOUND)

# - Try to find libTooN
#
#  TooN_FOUND - system has libTooN
#  TooN_INCLUDE_DIR - the libTooN include directories

FIND_PATH(
  TooN_INCLUDE_DIR
  NAMES TooN/TooN.h
  PATHS
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/..
    /usr/include
    /usr/local/include
)

IF(TooN_INCLUDE_DIR)
  SET(TooN_FOUND TRUE)
ENDIF()

IF(TooN_FOUND)
   IF(NOT TooN_FIND_QUIETLY)
      MESSAGE(STATUS "Found TooN: ${TooN_INCLUDE_DIR}")
   ENDIF()
ELSE()
   IF(TooN_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find TooN")
   ENDIF()
ENDIF()

# - Try to find Xrandr
#
#  Xrandr_FOUND - system has libXrandr
#  Xrandr_INCLUDE_DIRS - the libXrandr include directories
#  Xrandr_LIBRARIES - link these to use libXrandr

FIND_PATH(
  Xrandr_INCLUDE_DIRS
  NAMES X11/extensions/Xrandr.h
  PATH_SUFFIXES X11/extensions
  DOC "The Xrandr include directory"
)

FIND_LIBRARY(
  Xrandr_LIBRARIES
  NAMES Xrandr
  DOC "The Xrandr library"
)

IF (Xrandr_INCLUDE_DIRS AND Xrandr_LIBRARIES)
   SET(Xrandr_FOUND TRUE)
ENDIF (Xrandr_INCLUDE_DIRS AND Xrandr_LIBRARIES)

IF (Xrandr_FOUND)
   IF (NOT Xrandr_FIND_QUIETLY)
      MESSAGE(STATUS "Found Xrandr: ${Xrandr_LIBRARIES}")
   ENDIF (NOT Xrandr_FIND_QUIETLY)
ELSE (Xrandr_FOUND)
   IF (Xrandr_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Xrandr")
   ENDIF (Xrandr_FIND_REQUIRED)
ENDIF (Xrandr_FOUND)

# Try to find the OpenEXR v2 lib and include files
#
# OpenEXR_INCLUDE_DIR
# OpenEXR_LIBRARIES
# OpenEXR_FOUND

FIND_PATH( OpenEXR_INCLUDE_DIR ImfHeader.h
  /usr/include
  /usr/local/include
  PATH_SUFFIXES OpenEXR
)

FIND_LIBRARY( OpenEXR_LIBRARY IlmImf
  /usr/lib64
  /usr/lib
  /usr/local/lib
)

IF(OpenEXR_INCLUDE_DIR AND OpenEXR_LIBRARY)
  SET( OpenEXR_FOUND TRUE )
  SET( OpenEXR_LIBRARIES ${OpenEXR_LIBRARY} )
ENDIF()

IF(OpenEXR_FOUND)
   IF(NOT OpenEXR_FIND_QUIETLY)
      MESSAGE(STATUS "Found OpenEXR: ${OpenEXR_LIBRARY}")
   ENDIF(NOT OpenEXR_FIND_QUIETLY)
ELSE(OpenEXR_FOUND)
   IF(OpenEXR_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find libOpenEXR")
   ENDIF(OpenEXR_FIND_REQUIRED)
ENDIF(OpenEXR_FOUND)


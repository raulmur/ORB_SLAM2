# Try to find the DepthSense SDK For SoftKinetic Cameras
#
# DepthSense_INCLUDE_DIRS
# DepthSense_LIBRARIES
# DepthSense_FOUND

FIND_PATH( DepthSense_INCLUDE_DIR DepthSense.hxx
  PATHS
      "${PROGRAM_FILES}/SoftKinetic/DepthSenseSDK/include"
      "${PROGRAM_FILES}/Meta/DepthSenseSDK/include"
      /usr/include
      /usr/local/include
      /opt/local/include
      /opt/softkinetic/DepthSenseSDK/include
)

FIND_LIBRARY( DepthSense_LIBRARY DepthSense
  PATHS
      "${PROGRAM_FILES}/SoftKinetic/DepthSenseSDK/lib"
      "${PROGRAM_FILES}/Meta/DepthSenseSDK/lib"
      /usr/lib64
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /opt/softkinetic/DepthSenseSDK/lib
)

IF(DepthSense_INCLUDE_DIR AND DepthSense_LIBRARY)
  SET( DepthSense_FOUND TRUE )
  SET( DepthSense_LIBRARIES ${DepthSense_LIBRARY} )
  SET( DepthSense_INCLUDE_DIRS ${DepthSense_INCLUDE_DIR} )
ENDIF()

IF(DepthSense_FOUND)
   IF(NOT DepthSense_FIND_QUIETLY)
      MESSAGE(STATUS "Found DepthSense: ${DepthSense_LIBRARY}")
   ENDIF()
ELSE()
   IF(DepthSense_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find DepthSense")
   ENDIF()
ENDIF()


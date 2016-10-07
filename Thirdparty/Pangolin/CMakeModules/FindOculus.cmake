# - Try to find Oculus Rift SDK
#
#  Oculus_FOUND - system has libuvc
#  Oculus_INCLUDE_DIRS - the libuvc include directories
#  Oculus_LIBRARIES - link these to use libuvc

FIND_PATH(
  Oculus_INCLUDE_DIRS
  NAMES OVR.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../LibOVR/Include
    ${CMAKE_SOURCE_DIR}/../OculusSDK/LibOVR/Include
    /usr/include/LibOVR/Include
    /usr/local/include/LibOVR/Include
    /opt/local/include/LibOVR/Include
    /usr/include/
    /usr/local/include
    /opt/local/include
)

FIND_LIBRARY(
  Oculus_LIBRARIES
  NAMES ovr
  PATHS
    ${CMAKE_SOURCE_DIR}/../LibOVR/Lib/MacOS/Release
    ${CMAKE_SOURCE_DIR}/../OculusSDK/LibOVR/Lib/Linux/Release/x86_64
    /usr/include/LibOVR/Lib
    /usr/local/include/LibOVR/Lib
    /opt/local/include/LibOVR/Lib
    /usr/lib
    /usr/local/lib
    /opt/local/lib
)

IF(Oculus_INCLUDE_DIRS AND Oculus_LIBRARIES)
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        find_library(CARBON_LIBRARIES NAMES Carbon)
        find_library(IOKIT_LIBRARIES NAMES IOKit)
        list(APPEND Oculus_LIBRARIES ${CARBON_LIBRARIES})
        list(APPEND Oculus_LIBRARIES ${IOKIT_LIBRARIES})
        SET(Oculus_FOUND TRUE)
    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        FIND_PACKAGE(Xrandr QUIET)
        IF( Xrandr_FOUND )
            list(APPEND Oculus_LIBRARIES ${Xrandr_LIBRARIES} -ludev -lXrandr -lXinerama )
            SET(Oculus_FOUND TRUE)
        ENDIF()
    ENDIF()
ENDIF(Oculus_INCLUDE_DIRS AND Oculus_LIBRARIES)



IF(Oculus_FOUND)
   IF(NOT Oculus_FIND_QUIETLY)
      MESSAGE(STATUS "Found Oculus: ${Oculus_LIBRARIES}")
      MESSAGE(STATUS "Found Oculus: ${Oculus_INCLUDE_DIRS}")
   ENDIF(NOT Oculus_FIND_QUIETLY)
ELSE(Oculus_FOUND)
message(STATUS "Oculus NOT found")
   IF(Oculus_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Oculus")
   ENDIF(Oculus_FIND_REQUIRED)
ENDIF(Oculus_FOUND)

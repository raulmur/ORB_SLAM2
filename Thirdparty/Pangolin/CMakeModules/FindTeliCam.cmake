###############################################################################
# Find Toshiba TeliCam
#
# This sets the following variables:
# TeliCam_FOUND - True if TeliCam was found.
# TeliCam_INCLUDE_DIRS - Directories containing the TeliCam include files.
# TeliCam_LIBRARIES - Libraries needed to use TeliCam.

find_path(
    TeliCam_INCLUDE_DIR TeliCamApi.h
    PATHS
        "${PROGRAM_FILES}/Toshiba Teli/TeliCamSDK/TeliCamApi/Include"
        "${CMAKE_SOURCE_DIR}/../TeliCamSDK/TeliCamApi/Include"
        /usr/include
        /user/include
	/opt/TeliCamSDK/include
    PATH_SUFFIXES TeliCam
)

if(${CMAKE_CL_64})
    set(TELI_PATH_SUFFIXES x64)
else()
    set(TELI_PATH_SUFFIXES x86)
endif()

find_library(
    TeliCamApi_LIBRARY
    NAMES TeliCamApi TeliCamApi64 TeliCamApi_64
    PATHS
        "${PROGRAM_FILES}/Toshiba Teli/TeliCamSDK/TeliCamApi/lib"
        "${CMAKE_SOURCE_DIR}/../TeliCamSDK/TeliCamApi/lib"
        /usr/lib
        /user/lib
	/opt/TeliCamSDK/lib
    PATH_SUFFIXES ${TELI_PATH_SUFFIXES}
)

find_library(
    TeliCamUtl_LIBRARY
    NAMES TeliCamUtl TeliCamUtl64 TeliCamUtl_64
    PATHS
        "${PROGRAM_FILES}/Toshiba Teli/TeliCamSDK/TeliCamApi/lib"
        "${CMAKE_SOURCE_DIR}/../TeliCamSDK/TeliCamApi/lib"
        /usr/lib
        /user/lib
	/opt/TeliCamSDK/lib
    PATH_SUFFIXES ${TELI_PATH_SUFFIXES}
)

set(TeliCam_INCLUDE_DIRS ${TeliCam_INCLUDE_DIR})
set(TeliCam_LIBRARY "${TeliCamApi_LIBRARY}" "${TeliCamUtl_LIBRARY}")
set(TeliCam_LIBRARIES ${TeliCam_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args( TeliCam
  FOUND_VAR TeliCam_FOUND
  REQUIRED_VARS TeliCamApi_LIBRARY TeliCamUtl_LIBRARY TeliCam_INCLUDE_DIR
)

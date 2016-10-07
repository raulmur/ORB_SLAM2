###############################################################################
# Find OpenNI2
#
# This sets the following variables:
# OPENNI2_FOUND - True if OPENNI was found.
# OPENNI2_INCLUDE_DIRS - Directories containing the OPENNI include files.
# OPENNI2_LIBRARIES - Libraries needed to use OPENNI.

find_package(PkgConfig)
if(${CMAKE_VERSION} VERSION_LESS 2.8.2)
  pkg_check_modules(PC_OPENNI openni2-dev)
else()
  pkg_check_modules(PC_OPENNI QUIET openni2-dev)
endif()

set(OPENNI2_DEFINITIONS ${PC_OPENNI_CFLAGS_OTHER})

#add a hint so that it can find it without the pkg-config
find_path(OPENNI2_INCLUDE_DIR OpenNI.h
          HINTS
            ${PC_OPENNI_INCLUDEDIR}
            ${PC_OPENNI_INCLUDE_DIRS}
          PATHS
            "${PROGRAM_FILES}/OpenNI2/Include"
            "${CMAKE_SOURCE_DIR}/../OpenNI2/Include"
            /usr/include
            /user/include
          PATH_SUFFIXES openni2 ni2
)

if(${CMAKE_CL_64})
    set(OPENNI_PATH_SUFFIXES lib64 lib)
else()
    set(OPENNI_PATH_SUFFIXES lib)
endif()

#add a hint so that it can find it without the pkg-config
find_library(OPENNI2_LIBRARY
             NAMES OpenNI2
             HINTS
               ${PC_OPENNI_LIBDIR}
               ${PC_OPENNI_LIBRARY_DIRS}
             PATHS
               "${PROGRAM_FILES}/OpenNI2/Redist"
               "${PROGRAM_FILES}/OpenNI2"
               "${CMAKE_SOURCE_DIR}/../OpenNI2/Bin/x64-Release"
               /usr/lib
               /user/lib
             PATH_SUFFIXES ${OPENNI_PATH_SUFFIXES}
)

set(OPENNI2_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIR})
set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2 DEFAULT_MSG
    OPENNI2_LIBRARY OPENNI2_INCLUDE_DIR)

mark_as_advanced(OPENNI2_LIBRARY OPENNI2_INCLUDE_DIR)

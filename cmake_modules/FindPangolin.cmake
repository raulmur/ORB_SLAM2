# - Try to find Pangolin lib
#
# Once done this will define
#
#  PANGOLIN_FOUND - system found Pangolin
#  Pangolin_INCLUDE_DIRS - the Pangolin include directory
#  Pangolin_LIBRARIES - the Pangolin library directory

# Copyright (c) 2016, 2017 Yuanhui HE, <hyh618@gmail.com>
# Redistribution and use is allowed according to the terms of the BSD license.

# specific additional paths for some OS
if (WIN32)
  set(PANGOLIN_ADDITIONAL_SEARCH_PATHS ${PANGOLIN_ADDITIONAL_SEARCH_PATHS} "C:/Program Files/Pangolin/include" "C:/Program Files (x86)/Pangolin/include")
endif(WIN32)

find_path(Pangolin_INCLUDE_DIRS pangolin/pangolin.h
    PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/include
    HINTS ${PC_PANGOLIN__INCLUDEDIR} ${PC_PANGOLIN_INCLUDE_DIRS}
    PATH_SUFFIXES pangolin
  )

find_library(Pangolin_LIBRARIES
    NAMES pangolin libpangolin
    PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/lib
    HINTS ${PC_PANGOLIN_LIBDIR} ${PC_PANGOLIN_LIBRARY_DIRS}
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PANGOLIN DEFAULT_MSG Pangolin_INCLUDE_DIRS Pangolin_LIBRARIES)

mark_as_advanced(Pangolin_INCLUDE_DIRS)


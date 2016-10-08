# - Try to find OpenCV lib
#
# Once done this will define
#
#  OPENCV_FOUND - system found OpenCV
#  OpenCV_INCLUDE_DIRS - the OpenCV include directory
#  OpenCV_LIBRARIES - the OpenCV library directory

# Copyright (c) 2016, 2017 Yuanhui HE, <hyh618@gmail.com>
# Redistribution and use is allowed according to the terms of the BSD license.

# specific additional paths for some OS
if (WIN32)
  set(OpenCV_ADDITIONAL_SEARCH_PATHS ${OpenCV_ADDITIONAL_SEARCH_PATHS} "C:/Program Files/OpenCV/include" "C:/Program Files (x86)/OpenCV/include")
endif(WIN32)

find_path(OpenCV_INCLUDE_DIRS opencv2/opencv.hpp
    PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/opencv-2.4.13/include
    HINTS ${PC_PANGOLIN__INCLUDEDIR} ${PC_PANGOLIN_INCLUDE_DIRS}
    PATH_SUFFIXES opencv
  )

#find_library(OpenCV_LIBRARIES
#    NAMES opencv libopencv
#    PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/OpenCV/lib
#    HINTS ${PC_OPENCV_LIBDIR} ${PC_OPENCV_LIBRARY_DIRS}
#  )

include(FindPackageHandleStandardArgs)
#find_package_handle_standard_args(OPENCV DEFAULT_MSG OpenCV_INCLUDE_DIRS OpenCV_LIBRARIES)
find_package_handle_standard_args(OPENCV DEFAULT_MSG OpenCV_INCLUDE_DIRS)

mark_as_advanced(OpenCV_INCLUDE_DIRS)


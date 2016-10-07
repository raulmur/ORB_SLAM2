
if (GOTO_LIBRARIES)
  set(GOTO_FIND_QUIETLY TRUE)
endif (GOTO_LIBRARIES)

find_library(GOTO_LIBRARIES goto PATHS $ENV{GOTODIR} ${LIB_INSTALL_DIR})

if(GOTO_LIBRARIES AND CMAKE_COMPILER_IS_GNUCXX)
  set(GOTO_LIBRARIES ${GOTO_LIBRARIES} "-lpthread -lgfortran")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GOTO DEFAULT_MSG GOTO_LIBRARIES)

mark_as_advanced(GOTO_LIBRARIES)

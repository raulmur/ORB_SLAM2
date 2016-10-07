
if (GOTO2_LIBRARIES)
  set(GOTO2_FIND_QUIETLY TRUE)
endif (GOTO2_LIBRARIES)
# 
# find_path(GOTO_INCLUDES
#   NAMES
#   cblas.h
#   PATHS
#   $ENV{GOTODIR}/include
#   ${INCLUDE_INSTALL_DIR}
# )

find_file(GOTO2_LIBRARIES libgoto2.so PATHS /usr/lib $ENV{GOTO2DIR} ${LIB_INSTALL_DIR})
find_library(GOTO2_LIBRARIES goto2 PATHS $ENV{GOTO2DIR} ${LIB_INSTALL_DIR})

if(GOTO2_LIBRARIES AND CMAKE_COMPILER_IS_GNUCXX)
  set(GOTO2_LIBRARIES ${GOTO2_LIBRARIES} "-lpthread -lgfortran")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GOTO2 DEFAULT_MSG
                                  GOTO2_LIBRARIES)

mark_as_advanced(GOTO2_LIBRARIES)

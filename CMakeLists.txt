cmake_minimum_required(VERSION 2.8)

project( top_level )

get_filename_component( BUILD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/build  ABSOLUTE )
get_filename_component( DEVEL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/devel ABSOLUTE )
get_filename_component( GLOBAL_CMAKE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/devel/cmake ABSOLUTE )
get_filename_component( GLOBAL_INCLUDE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/devel/include ABSOLUTE )
get_filename_component( GLOBAL_LIB_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/devel/lib ABSOLUTE )
get_filename_component( GLOBAL_BIN_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/devel/bin ABSOLUTE )

set( LIBRARY_OUTPUT_PATH ${GLOBAL_LIB_ROOT} )
# set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${GLOBAL_BIN_ROOT} )

list( APPEND CMAKE_MODULE_PATH ${CMAKE_GLOBAL_ROOT}/modules )

add_definitions("-w")

function(set_vars_and_add_subdirectory SRC_ROOT PROJECT_NAME)
    message( "BUILDING PROJECT: ${PROJECT_NAME}" )
    make_directory( ${GLOBAL_BIN_ROOT}/${PROJECT_NAME} )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${GLOBAL_BIN_ROOT}/${PROJECT_NAME} )
    add_subdirectory( ${SRC_ROOT} )
    file( COPY ${SRC_ROOT}/include DESTINATION ${DEVEL_ROOT} )
endfunction(set_vars_and_add_subdirectory)

set_vars_and_add_subdirectory( src/ORB_SLAM2/DBoW2 DBoW2 )
# file( COPY src/ORB_SLAM2/DBoW2/include DESTINATION ${DEVEL_ROOT} )
# copy_dirs_to_devel( src/ORB_SLAM2/DBoW2 )

set_vars_and_add_subdirectory( src/ORB_SLAM2/g2o g2o )
# file( COPY src/ORB_SLAM2/g2o/include DESTINATION ${DEVEL_ROOT} )
# copy_dirs_to_devel( src/ORB_SLAM2/g2o )

set_vars_and_add_subdirectory( src/ORB_SLAM2 ORB_SLAM2 )
# file( COPY src/ORB_SLAM2/include DESTINATION ${DEVEL_ROOT} )
# copy_dirs_to_devel( src/ORB_SLAM2 )

set_vars_and_add_subdirectory( src/orbslampy orbslampy )
# file( COPY src/orbslampy/include DESTINATION ${DEVEL_ROOT} )
# copy_dirs_to_devel( src/orbslampy )

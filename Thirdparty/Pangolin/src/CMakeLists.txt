#######################################################
## Library sources

macro( append_glob list glob )
    file(GLOB files ${glob})
    set(${list} "${${list}};${files}")
endmacro()


## Header only includes / core headers
set( INCDIR "../include/pangolin" )
set( HEADERS
  ${INCDIR}/pangolin.h
  ${INCDIR}/platform.h
)
append_glob(HEADERS ${INCDIR}/utils/*.h*)
append_glob(HEADERS ${INCDIR}/image/*.h*)
append_glob(HEADERS ${INCDIR}/log/*.h*)

### Store list of source files
append_glob(SOURCES utils/*.cpp)
append_glob(SOURCES image/*.cpp)
append_glob(SOURCES log/*.cpp)

#######################################################
## User build options

option(BUILD_PANGOLIN_GUI "Build support for Pangolin GUI" ON)
if(BUILD_PANGOLIN_GUI)
  append_glob(HEADERS ${INCDIR}/gl/*.h*)
  append_glob(HEADERS ${INCDIR}/display/*.h*)
  append_glob(HEADERS ${INCDIR}/handler/*.h*)

  append_glob(SOURCES gl/*.cpp)
  append_glob(SOURCES display/*.cpp)
  append_glob(SOURCES handler/*.cpp)

  if(NOT HAVE_GLES OR HAVE_GLES_2)
    append_glob(HEADERS ${INCDIR}/plot/*.h*)
    append_glob(SOURCES plot/*.cpp)
  endif()

endif()

option(BUILD_PANGOLIN_VARS "Build support for Pangolin Vars" ON)
if(BUILD_PANGOLIN_VARS)
  append_glob(HEADERS ${INCDIR}/var/*.h*)
  append_glob(SOURCES var/*.cpp)

  if(BUILD_PANGOLIN_GUI)
      list(APPEND HEADERS ${INCDIR}/display/widgets/widgets.h )
      list(APPEND SOURCES display/widgets/widgets.cpp )
  endif()
endif()

option(BUILD_PANGOLIN_VIDEO "Build support for Pangolin Video Utilities" ON)
if(BUILD_PANGOLIN_VIDEO)
  # Generic video includes
  append_glob(HEADERS ${INCDIR}/video/*.h*)
  append_glob(SOURCES video/*.cpp)

  # Generic video drivers
  list(APPEND HEADERS
    ${INCDIR}/video/drivers/test.h
    ${INCDIR}/video/drivers/images.h
    ${INCDIR}/video/drivers/video_splitter.h
    ${INCDIR}/video/drivers/pvn_video.h
    ${INCDIR}/video/drivers/pango_video.h
    ${INCDIR}/video/drivers/pango_video_output.h
    ${INCDIR}/video/drivers/debayer.h
    ${INCDIR}/video/drivers/shift.h
    ${INCDIR}/video/drivers/mirror.h
    ${INCDIR}/video/drivers/unpack.h
    ${INCDIR}/video/drivers/join.h
    ${INCDIR}/video/drivers/thread.h
  )
  list(APPEND SOURCES
    video/drivers/test.cpp
    video/drivers/images.cpp
    video/drivers/video_splitter.cpp
    video/drivers/pvn_video.cpp
    video/drivers/pango_video.cpp
    video/drivers/pango_video_output.cpp
    video/drivers/debayer.cpp
    video/drivers/shift.cpp
    video/drivers/mirror.cpp
    video/drivers/unpack.cpp
    video/drivers/join.cpp
    video/drivers/thread.cpp
  )

  if(UNIX)
    list(APPEND HEADERS ${INCDIR}/video/drivers/shared_memory.h)
    list(APPEND SOURCES video/drivers/shared_memory.cpp)
    if(_LINUX_)
      # Required for shared memory API using some versions of glibc
      list(APPEND LINK_LIBS rt pthread)
    endif()
  endif()

endif()

#######################################################
## Setup required includes / link info

# Project headers trump everything (including any potentially installed Pangolin)
list(APPEND LIB_INC_DIR  "${PROJECT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}/include" )

if(NOT CPP11_NO_BOOST)
    # Boost is required when c++11 is disabled
    find_package(Boost COMPONENTS thread filesystem system REQUIRED QUIET)
    list(APPEND USER_INC  "${Boost_INCLUDE_DIRS}" )
    list(APPEND LINK_LIBS "${Boost_LIBRARIES}" )
endif()

if(BUILD_PANGOLIN_GUI)
    if( ANDROID  )
        # Android specific display code
        list(APPEND HEADERS ${INCDIR}/display/device/display_android.h )
        list(APPEND SOURCES display/device/display_android.cpp )

        if(HAVE_GLES_2)
            list(APPEND LINK_LIBS "-lEGL;-lGLESv2" )
        else()
            list(APPEND LINK_LIBS "-lEGL;-lGLESv1_CM" )
        endif()
    elseif( IOS )
        list(APPEND LINK_LIBS "-framework OpenGLES" )
        list(APPEND HEADERS "${INCDIR}/ios/PangolinAppDelegate.h" "${INCDIR}/ios/PangolinViewController.h" )
        list(APPEND SOURCES "ios/PangolinAppDelegate.mm" "ios/PangolinViewController.mm" )
    else()
        find_package(OpenGL REQUIRED QUIET)
        list(APPEND USER_INC  "${OPENGL_INCLUDE_DIR}" )
        list(APPEND LINK_LIBS "${OPENGL_LIBRARIES}" )

        if(NOT BUILD_EXTERN_GLEW)
            find_package(GLEW REQUIRED QUIET)
        endif()
        if(GLEW_FOUND)
            list(APPEND USER_INC  "${GLEW_INCLUDE_DIR}" )
            list(APPEND LINK_LIBS "${GLEW_LIBRARY}" )
            set(HAVE_GLEW 1)
        endif()
    endif()

    if( HAVE_GLES_2 )
        # Add Pangolins backwards compat layer.
        list(APPEND HEADERS ${INCDIR}/gl2engine.h )
        list(APPEND SOURCES gl2engine.cpp)
    endif()
endif()

#######################################################
## Find optional dependencies

if(ANDROID)
    # Fix issue with thread local storage on android.
    add_definitions(-fno-data-sections)
    list(APPEND LINK_LIBS android log)
elseif(IOS)
    # Nothing specific in here yet.
else()
    if(BUILD_PANGOLIN_GUI)
        if(FORCE_GLUT)
            find_package(FREEGLUT QUIET)
            find_package(GLUT QUIET)

            if(_OSX_)
              INCLUDE(CheckCXXSourceRuns)
              set(CMAKE_REQUIRED_LIBRARIES ${GLUT_LIBRARY})
              CHECK_CXX_SOURCE_RUNS("#include <GLUT/glut.h>\n int main () {return glutGetProcAddress(\"glutScrollFunc\") ? 0 : -1;};" HAVE_MODIFIED_OSXGLUT)
              if(NOT HAVE_MODIFIED_OSXGLUT)
                    message(STATUS "Install modified osxglut for smooth scroll support / pinch / zoom.")
                    message(STATUS "(https://github.com/stevenlovegrove/osxglut)")
              endif()
            endif()

            # Prefer OSX_MODIFIED_GLUT > FREEGLUT > GLUT
            if(FREEGLUT_FOUND AND NOT HAVE_MODIFIED_OSXGLUT)
              set(HAVE_FREEGLUT 1)
              list(APPEND USER_INC  ${FREEGLUT_INCLUDE_DIR} )
              list(APPEND LINK_LIBS ${FREEGLUT_LIBRARY} )
            elseif(GLUT_FOUND)
              list(APPEND USER_INC  ${GLUT_INCLUDE_DIR} )
              list(APPEND LINK_LIBS ${GLUT_LIBRARY} )
              if(_OSX_)
                set(HAVE_APPLE_OPENGL_FRAMEWORK 1)
                if(NOT HAVE_MODIFIED_OSXGLUT)
                    message(STATUS "Using Apple Framework GLUT.")
                    message(STATUS "Install Freeglut or modified osxglut for scroll support.")
                endif()
              endif()
            endif()

            if(FREEGLUT_FOUND OR GLUT_FOUND)
                set(HAVE_GLUT 1)
                list(APPEND HEADERS ${INCDIR}/display/device/display_glut.h )
                list(APPEND SOURCES display/device/display_glut.cpp )
                message(STATUS "Glut Found and Enabled")
            endif()
        elseif(_WIN_)
            list(APPEND SOURCES display/device/display_win.cpp )
        elseif(_OSX_)
            list(APPEND SOURCES display/device/display_osx.mm display/device/PangolinNSApplication.mm display/device/PangolinNSGLView.mm  )
            list(APPEND LINK_LIBS "-framework Cocoa" )
        elseif(_LINUX_)
            find_package(X11 REQUIRED)
            list(APPEND USER_INC ${X11_INCLUDE_DIR})
            list(APPEND SOURCES display/device/display_x11.cpp )
            list(APPEND LINK_LIBS ${X11_LIBRARIES} )
        endif()
    endif()
endif()

if(UNIX)
  append_glob(HEADERS ${INCDIR}/utils/posix/*.h*)
  append_glob(SOURCES utils/posix/*.cpp)
endif()

find_package(PythonLibs QUIET)
if(BUILD_PANGOLIN_GUI AND BUILD_PANGOLIN_VARS AND PYTHONLIBS_FOUND AND NOT _WIN_)
  set(HAVE_PYTHON 1)
  list(APPEND HEADERS
    ${INCDIR}/console/ConsoleInterpreter.h
    ${INCDIR}/console/ConsoleView.h
    ${INCDIR}/python/PyInterpreter.h
    ${INCDIR}/python/PyModulePangolin.h
    ${INCDIR}/python/PyUniqueObj.h
    ${INCDIR}/python/PyVar.h
    ${INCDIR}/python/PyPangoIO.h
  )
  list(APPEND SOURCES
    console/ConsoleView.cpp
    python/PyInterpreter.cpp
    python/PyModulePangolin.cpp
  )
  if(_GCC_)
    set_source_files_properties(python/PyInterpreter.cpp PROPERTIES COMPILE_FLAGS -fno-strict-aliasing)
  endif()
  list(APPEND INTERNAL_INC ${PYTHON_INCLUDE_DIR})
  list(APPEND LINK_LIBS    ${PYTHON_LIBRARY})
  message(STATUS "Python Found and Enabled")
endif()

option(BUILD_PANGOLIN_OCULUS "Build support for Oculus HUD" OFF)
if(BUILD_PANGOLIN_OCULUS)
  find_package(Oculus QUIET)
  if(Oculus_FOUND)
    set(HAVE_OCULUS 1)
    list(APPEND HEADERS ${INCDIR}/hud/oculus_hud.h )
    list(APPEND SOURCES hud/oculus_hud.cpp )
    list(APPEND USER_INC ${Oculus_INCLUDE_DIRS} )
    list(APPEND LINK_LIBS ${Oculus_LIBRARIES} )
    message(STATUS "Oculus Rift Found and Enabled")
  endif()
endif()

find_package(Eigen3 QUIET)
if(EIGEN3_FOUND)
  set(HAVE_EIGEN 1)
  list(APPEND USER_INC  ${EIGEN3_INCLUDE_DIR} )
  message(STATUS "Eigen Found and Enabled")
endif()

find_package(TooN QUIET)
if(TooN_FOUND)
  set(HAVE_TOON 1)
  list(APPEND USER_INC  ${TooN_INCLUDE_DIR} )
  message(STATUS "TooN Found and Enabled")
endif()

find_package(DC1394 QUIET)
if(BUILD_PANGOLIN_VIDEO AND DC1394_FOUND)
  set(HAVE_DC1394 1)
  list(APPEND INTERNAL_INC  ${DC1394_INCLUDE_DIR} )
  list(APPEND LINK_LIBS  ${DC1394_LIBRARY} )
  list(APPEND HEADERS ${INCDIR}/video/drivers/firewire.h ${INCDIR}/video/drivers/firewire_deinterlace.h )
  list(APPEND SOURCES video/drivers/firewire.cpp video/drivers/firewire_deinterlace.cpp)
  message(STATUS "libdc1394 Found and Enabled")
endif()

if(BUILD_PANGOLIN_VIDEO AND _LINUX_)
  set(HAVE_V4L 1)
  list(APPEND HEADERS ${INCDIR}/video/drivers/v4l.h)
  list(APPEND SOURCES video/drivers/v4l.cpp)
  message(STATUS "V4L Found and Enabled")
endif()

find_package(FFMPEG QUIET)
if(BUILD_PANGOLIN_VIDEO AND FFMPEG_FOUND)
  set(HAVE_FFMPEG 1)
  list(APPEND INTERNAL_INC  ${FFMPEG_INCLUDE_DIRS} )
  list(APPEND LINK_LIBS ${FFMPEG_LIBRARIES} )
  list(APPEND HEADERS ${INCDIR}/video/drivers/ffmpeg.h)
  list(APPEND SOURCES video/drivers/ffmpeg.cpp)
  message(STATUS "ffmpeg Found and Enabled")
endif()

find_package(OpenNI QUIET)
if(BUILD_PANGOLIN_VIDEO AND OPENNI_FOUND)
  set(HAVE_OPENNI 1)
  if(_LINUX_)
    add_definitions(-Dlinux=1)
  endif()
  list(APPEND INTERNAL_INC  ${OPENNI_INCLUDE_DIRS} )
  list(APPEND LINK_LIBS ${OPENNI_LIBRARIES} )
  list(APPEND HEADERS ${INCDIR}/video/drivers/openni.h )
  list(APPEND SOURCES video/drivers/openni.cpp)
  message(STATUS "OpenNI Found and Enabled")
endif()

find_package(OpenNI2 QUIET)
if(BUILD_PANGOLIN_VIDEO AND OPENNI2_FOUND)
  set(HAVE_OPENNI2 1)
  if(_LINUX_)
    add_definitions(-Dlinux=1)
  endif()
  list(APPEND INTERNAL_INC  ${OPENNI2_INCLUDE_DIRS} )
  list(APPEND LINK_LIBS ${OPENNI2_LIBRARIES} )
  list(APPEND HEADERS ${INCDIR}/video/drivers/openni2.h )
  list(APPEND SOURCES video/drivers/openni2.cpp)
  message(STATUS "OpenNI2 Found and Enabled")
endif()

find_package(uvc QUIET)
if(BUILD_PANGOLIN_VIDEO AND uvc_FOUND)
  set(HAVE_UVC 1)
  list(APPEND INTERNAL_INC  ${uvc_INCLUDE_DIRS} )
  list(APPEND LINK_LIBS ${uvc_LIBRARIES} )
  list(APPEND HEADERS ${INCDIR}/video/drivers/uvc.h )
  list(APPEND SOURCES video/drivers/uvc.cpp)
  if(_WIN_)
    find_package(pthread REQUIRED QUIET)
    list(APPEND LINK_LIBS ${pthread_LIBRARIES} )

    find_package(libusb1 REQUIRED QUIET)
    list(APPEND LINK_LIBS ${libusb1_LIBRARIES} )
  endif()
  message(STATUS "libuvc Found and Enabled")
endif()

find_package(DepthSense QUIET)
if(BUILD_PANGOLIN_VIDEO AND DepthSense_FOUND)
  set(HAVE_DEPTHSENSE 1)
  list(APPEND INTERNAL_INC  ${DepthSense_INCLUDE_DIRS} )
  list(APPEND LINK_LIBS ${DepthSense_LIBRARIES} )
  list(APPEND HEADERS ${INCDIR}/video/drivers/depthsense.h )
  list(APPEND SOURCES video/drivers/depthsense.cpp)
  message(STATUS "DepthSense Found and Enabled")
endif()

find_package(TeliCam QUIET)
if(BUILD_PANGOLIN_VIDEO AND TeliCam_FOUND)
  set(HAVE_TELICAM 1)
  list(APPEND INTERNAL_INC ${TeliCam_INCLUDE_DIRS} )
  list(APPEND LINK_LIBS ${TeliCam_LIBRARIES} )
  list(APPEND HEADERS ${INCDIR}/video/drivers/teli.h )
  list(APPEND SOURCES video/drivers/teli.cpp )
  message(STATUS "TeliCam Found and Enabled" )
endif()

find_package(Pleora QUIET)
if(BUILD_PANGOLIN_VIDEO AND Pleora_FOUND)
  set(HAVE_PLEORA 1)
  list(APPEND INTERNAL_INC ${Pleora_INCLUDE_DIRS} )
  list(APPEND LINK_LIBS ${Pleora_LIBRARIES} )
  list(APPEND HEADERS ${INCDIR}/video/drivers/pleora.h )
  list(APPEND SOURCES video/drivers/pleora.cpp )
  if(_GCC_)
    # Suppress warnings generated from Pleora SDK.
    set_source_files_properties(video/drivers/pleora.cpp PROPERTIES COMPILE_FLAGS -Wno-unknown-pragmas)
    set_source_files_properties(video/video.cpp PROPERTIES COMPILE_FLAGS -Wno-unknown-pragmas)
  endif()
  message(STATUS "Pleora Found and Enabled" )
endif()

if(NOT BUILD_EXTERN_LIBPNG)
  find_package(PNG QUIET)
endif()
if(PNG_FOUND)
  # (ZLIB is also found by FindPNG.cmake as its dependency)
  set(HAVE_PNG 1)
  list(APPEND INTERNAL_INC ${PNG_INCLUDE_DIR} )
  list(APPEND LINK_LIBS ${PNG_LIBRARY} ${ZLIB_LIBRARY} )
  message(STATUS "libpng Found and Enabled")
endif()

if(NOT BUILD_EXTERN_LIBJPEG)
  find_package(JPEG QUIET)
endif()
if(JPEG_FOUND)
  set(HAVE_JPEG 1)
  list(APPEND INTERNAL_INC ${JPEG_INCLUDE_DIR} )
  list(APPEND LINK_LIBS ${JPEG_LIBRARY} )
  message(STATUS "libjpeg Found and Enabled")
endif()

find_package(TIFF QUIET)
if(TIFF_FOUND)
  set(HAVE_TIFF 1)
  list(APPEND INTERNAL_INC ${TIFF_INCLUDE_DIR} )
  list(APPEND LINK_LIBS ${TIFF_LIBRARY} )
  message(STATUS "libtiff Found and Enabled")
endif()

find_package(OpenEXR QUIET)
if(OpenEXR_FOUND)
  set(HAVE_OPENEXR 1)
  list(APPEND INTERNAL_INC ${OpenEXR_INCLUDE_DIR} )
  list(APPEND LINK_LIBS ${OpenEXR_LIBRARY} )
  message(STATUS "libopenexr Found and Enabled")
endif()


#######################################################
## Embed resource binary files

include(EmbedBinaryFiles)

if(BUILD_PANGOLIN_GUI)
    embed_binary_files( "_embed_/fonts/*.ttf" "${CMAKE_CURRENT_BINARY_DIR}/fonts.c" )
    list(APPEND SOURCES "${CMAKE_CURRENT_BINARY_DIR}/fonts.c" )
endif()

#######################################################
## Add Libraries / Include Directories / Link directories

include_directories( ${LIB_INC_DIR} )
include_directories( ${USER_INC} )
include_directories( ${INTERNAL_INC} )

add_library(${LIBRARY_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${LIBRARY_NAME} ${LINK_LIBS})

## Generate symbol export helper header on MSVC
IF(MSVC)
    string(TOUPPER ${LIBRARY_NAME} LIBRARY_NAME_CAPS)
    include(GenerateExportHeader)
    GENERATE_EXPORT_HEADER( ${LIBRARY_NAME}
        BASE_NAME ${LIBRARY_NAME_CAPS}
        EXPORT_MACRO_NAME ${LIBRARY_NAME_CAPS}_EXPORT
        EXPORT_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/include/${LIBRARY_NAME}/${LIBRARY_NAME}_export.h"
        STATIC_DEFINE ${LIBRARY_NAME_CAPS}_BUILT_AS_STATIC
    )
ENDIF()

## Set Special Compiler flags
if(MSVC)
  set(CMAKE_CXX_FLAGS "/EHs ${CMAKE_CXX_FLAGS}")
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-Wall -Wno-error=deprecated-declarations ${CMAKE_CXX_FLAGS}")
endif()

#######################################################
## Create config.h file for inclusion in library

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/${LIBRARY_NAME}/config.h"
)

#######################################################
## Generate Doxygen documentation target (make doc)
find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../doc )
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../doc
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif()

#######################################################

# This relative path allows installed files to be relocatable.
set( CMAKECONFIG_INSTALL_DIR lib/cmake/${PROJECT_NAME} )
file( RELATIVE_PATH REL_INCLUDE_DIR
    "${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR}"
    "${CMAKE_INSTALL_PREFIX}/include"
)

# Export library for easy inclusion from other cmake projects. APPEND allows
# call to function even as subdirectory of larger project.
FILE(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")
export( TARGETS ${LIBRARY_NAME}
        APPEND FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake" )

# Version information
configure_file(${PROJECT_NAME}ConfigVersion.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)

# Build tree config
set( EXPORT_LIB_INC_DIR ${LIB_INC_DIR} )
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY IMMEDIATE )

# Install tree config
set( EXPORT_LIB_INC_DIR "\${PROJECT_CMAKE_DIR}/${REL_INCLUDE_DIR}" )
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake @ONLY )

# Add package to CMake package registery for use from the build tree
option( EXPORT_${PROJECT_NAME}
  "Should the ${PROJECT_NAME} package be exported for use by other software" ON )

if( EXPORT_${PROJECT_NAME} )
  export( PACKAGE ${PROJECT_NAME} )
endif()

#######################################################
## Install headers / targets

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/include/${LIBRARY_NAME}/config.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${LIBRARY_NAME}
)
install(DIRECTORY ${INCDIR}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)
install(TARGETS ${LIBRARY_NAME}
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

#######################################################
## Install CMake config

install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)
install(
    EXPORT ${PROJECT_NAME}Targets DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

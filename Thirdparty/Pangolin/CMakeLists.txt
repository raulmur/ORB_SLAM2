cmake_minimum_required(VERSION 2.6)
project("Pangolin")
set(PANGOLIN_VERSION_MAJOR 0)
set(PANGOLIN_VERSION_MINOR 4)
set(PANGOLIN_VERSION ${PANGOLIN_VERSION_MAJOR}.${PANGOLIN_VERSION_MINOR})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

# Platform configuration vars
include(SetPlatformVars)

option( BUILD_EXAMPLES "Build Examples" ON )
option( CPP11_NO_BOOST "Use c++11 over boost for threading etc." ON )

if(_WIN_)
    option( BUILD_SHARED_LIBS    "Build Shared Library" OFF)
    option( BUILD_EXTERN_GLEW    "Automatically download, build and compile GLEW" ON)
    option( BUILD_EXTERN_LIBPNG  "Automatically download, build and compile libpng" ON)
    option( BUILD_EXTERN_LIBJPEG "Automatically download, build and compile libjpeg" ON)
    option( MSVC_USE_STATIC_CRT  "Use static C Runtime with MSVC, /MT instead of /MD" ON)

    # Make sure there are no erroneous C Runtime flags
    list(APPEND FLAG_VARS
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO
        CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
    )
    if(MSVC_USE_STATIC_CRT)
        foreach(FLAG_VAR ${FLAG_VARS})
            string(REGEX REPLACE "/MD" "/MT" NEW_FLAGS "${${FLAG_VAR}}")
            set(${FLAG_VAR} "${NEW_FLAGS}" CACHE STRING "" FORCE)
        endforeach()
    else()
        foreach(FLAG_VAR ${FLAG_VARS})
            string(REGEX REPLACE "/MT" "/MD" NEW_FLAGS "${${FLAG_VAR}}")
            set(${FLAG_VAR} "${NEW_FLAGS}" CACHE STRING "" FORCE)
        endforeach()
    endif()
else()
    option( BUILD_SHARED_LIBS "Build Shared Library" ON)
endif()

if(CPP11_NO_BOOST AND NOT MSVC)
    set( CMAKE_CXX_FLAGS "-std=c++0x -Wall ${CMAKE_CXX_FLAGS}" )
    if(_CLANG_)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
    endif()
endif()

if(ANDROID)
    set(ANDROID_PACKAGE_NAME "com.github.stevenlovegrove.pangolin")
    include(AndroidUtils)
endif()

if(ANDROID OR IOS)
    set(HAVE_GLES 1)
    option(BUILD_FOR_GLES_2 "Build for OpenGL ES 2 instead of ES 1" ON )
    if(BUILD_FOR_GLES_2)
        set(HAVE_GLES_2 1)
    endif()
endif()

if(_OSX_)
    set(CMAKE_MACOSX_RPATH ON)
endif()

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
if( NOT CMAKE_BUILD_TYPE AND NOT _WIN_ )
   message("Build type not set (defaults to release)")
   message("-DCMAKE_BUILD_TYPE=Debug for debug")
   set( CMAKE_BUILD_TYPE Release )
endif()

string(TOLOWER ${PROJECT_NAME} LIBRARY_NAME)

# make an uninstall target 
configure_file(
 	"${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  	"${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  	IMMEDIATE @ONLY
)

add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

add_subdirectory("external")
add_subdirectory("src")

if(BUILD_EXAMPLES)
  set(Pangolin_DIR ${Pangolin_BINARY_DIR}/src)
  add_subdirectory(examples)
  add_subdirectory(tools)
endif()

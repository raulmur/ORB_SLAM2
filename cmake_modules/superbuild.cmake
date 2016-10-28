include(ExternalProject)

# Base directory for storing components of external projects
set_property(DIRECTORY PROPERTY EP_BASE Dependencies)

# Seqan Library, see http://seqan.de
set(PANGOLIN_VERSION "0.4")
set(PANGOLIN_MD5 "0b685ff92bbde1a010896735eb54f5fc")
ExternalProject_Add(ep_pangolin
    URL "https://github.com/stevenlovegrove/Pangolin/archive/v${PANGOLIN_VERSION}.tar.gz"
    URL_MD5 "${PANGOLIN_MD5}"
    PATCH_COMMAND patch -p 1 -i "${PROJECT_SOURCE_DIR}/cmake_modules/pangolin_${PANGOLIN_VERSION}.patch"
    CMAKE_ARGS
        "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        -DCPP11_NO_BOOST=1
)

set(EIGEN_VERSION "3.2.9")
set(EIGEN_MD5 "de11bfbfe2fd2dc4b32e8f416f58ee98")
ExternalProject_add(ep_eigen
    URL "http://bitbucket.org/eigen/eigen/get/${EIGEN_VERSION}.tar.bz2"
    URL_MD5 "${EIGEN_MD5}"
    CMAKE_ARGS
        "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        -DEIGEN_TEST_NO_OPENGL=ON
)

set(OPENCV_VERSION "2.4.13")
set(OPENCV_MD5 "8feb45a71adad89b8017a777477c3eff")
ExternalProject_add(ep_opencv
    URL "https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz"
    URL_MD5 "8feb45a71adad89b8017a777477c3eff"
    PATCH_COMMAND patch -p 1 -i "${PROJECT_SOURCE_DIR}/cmake_modules/opencv_${OPENCV_VERSION}.patch"
    CMAKE_ARGS
        "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        "-DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/Dependencies/Install/ep_eigen"
        -DWITH_CUDA=OFF
    DEPENDS ep_eigen
)

ExternalProject_add(ep_dbow2
    LIST_SEPARATOR :
    SOURCE_DIR "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2"
    CMAKE_ARGS
        "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/Dependencies/Install/ep_eigen:${CMAKE_CURRENT_BINARY_DIR}/Dependencies/Install/ep_opencv"
    INSTALL_COMMAND ""
    DEPENDS ep_eigen ep_opencv
)

ExternalProject_add(ep_g2o
    SOURCE_DIR "${PROJECT_SOURCE_DIR}/Thirdparty/g2o"
    CMAKE_ARGS
        "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/Dependencies/Install/ep_eigen"
    INSTALL_COMMAND ""
    DEPENDS ep_eigen
)

ExternalProject_add(ep_orbslam2
    SOURCE_DIR "${PROJECT_SOURCE_DIR}"
    CMAKE_ARGS
        "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
        -DDO_SUPERBUILD=OFF
        "-DDEPENDENCIES_PATH=${CMAKE_CURRENT_BINARY_DIR}/Dependencies"
    INSTALL_COMMAND ""
    DEPENDS ep_pangolin ep_eigen ep_opencv ep_dbow2 ep_g2o
)

#!/bin/bash

CurDir=$(dirname $0)
cmake_latest=/opt/cmake-3.9.2/bin/cmake
cd ${CurDir}
OrbSlamPlatform=`uname -m`
OrbSlamToolset=gcc.`gcc -dumpversion`

if [ ! -z "$1" ] 
then
    OrbSlamPlatform="$1"
fi

if [ ! -z "$2" ] 
then
    OrbSlamToolset="$2"
fi

if [ ! -z "$3" ] 
then
    OrbSlamBuildtype="$3"
fi

cd Vocabulary
if [ ! -e ORBvoc.txt ]
then
    echo "Uncompress vocabulary ..."
    tar -xf ORBvoc.txt.tar.gz
fi
cd ..

echo "Configuring and building Thirdparty/DBoW2 Thirdparty/g2o ORB_SLAM2 ..."

BuildDir="products/cmake.make.linux.${OrbSlamPlatform}.${OrbSlamToolset}"
if [ ! -e ${BuildDir} ] 
then 
	mkdir -p "${BuildDir}"
fi

if [ ! -e ${cmake_latest} ] 
then 
	cmake_latest=cmake
fi

${cmake_latest} . -B${BuildDir} \
-DCMAKE_BUILD_TYPE=${OrbSlamBuildtype} \
-DORBSLAM2_STATIC_LIB=ON \
-DG2O_STATIC_LIB=ON \
-DDBOW2_STATIC_LIB=ON \
-DBUILD_EXAMPLES=ON \
-DBUILD_THIRDPARTY_LIB=ON \
-DCMAKE_INSTALL_PREFIX=/usr/local

${cmake_latest} --build ${BuildDir} --target install


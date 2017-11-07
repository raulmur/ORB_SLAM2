#!/bin/bash

CurDir=$(dirname $0)
cd ${CurDir}
OrbSlamPlatform=`uname -m`
OrbSlamToolset=gcc.`gcc -dumpversion`
OrbSlamBuildtype=Debug

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

echo "Configuring and building Thirdparty/DBoW2 Thirdparty/g2o ORB_SLAM2 ..."

BuildDir="products/cmake.make.linux.${OrbSlamPlatform}.${OrbSlamToolset}.${OrbSlamBuildtype}"
if [ ! -e ${BuildDir} ] 
then 
	mkdir -p "${BuildDir}"
fi

if [ ! -e ${cmake_latest} ] 
then 
	cmake_latest=cmake
fi

cmake . -B${BuildDir} \
    -DCMAKE_BUILD_TYPE=${OrbSlamBuildtype} \
    -DBUILD_EXAMPLES=ON \
    -DCMAKE_INSTALL_PREFIX=/usr/local

cmake --build ${BuildDir} --target install

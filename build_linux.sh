#!/bin/bash

CurDir=$(dirname $0)
source ${CurDir}/bootstrap_linux.sh "$@"

OrbSlamPlatform=`uname -m`
OrbSlamToolset=gcc.`gcc -dumpversion`
Buildtype=Release

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
    Buildtype="$3"
fi

echo "Uncompress vocabulary ..."
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building Thirdparty/DBoW2 Thirdparty/g2o ORB_SLAM2 ..."

BuildDir="products/cmake.make.linux.${OrbSlamPlatform}.${OrbSlamToolset}"
if [ ! -e ${BuildDir} ] 
then 
	mkdir -p "${BuildDir}"
fi

cd ${BuildDir}

cmake ../.. -DCMAKE_BUILD_TYPE=Release -DORBSLAM2_STATIC_LIB=ON -DG2O_STATIC_LIB=ON -DDBOW2_STATIC_LIB=ON -DBUILD_EXAMPLES=ON -DBUILD_THIRDPARTY_LIB=ON

cmake --build .

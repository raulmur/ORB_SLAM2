#!/bin/bash

ScriptDir=$(realpath $(dirname $0))
ProjectDir=$(realpath ${ScriptDir}/../..)
cd ${ProjectDir}

OrbSlamPlatform=`uname -m`
OrbSlamToolset=gcc.`gcc -dumpversion`
OrbSlamBuildtype=Release
cores=$(nproc)

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

BuildDir="products/cmake.make.linux.${OrbSlamPlatform}.${OrbSlamToolset}.${OrbSlamBuildtype}"

mkdir -p "${BuildDir}"

cmake ${ProjectDir} \
    -B${BuildDir} \
    -DCMAKE_BUILD_TYPE=${OrbSlamBuildtype} \
    -DBUILD_EXAMPLES=ON \
    -DCMAKE_INSTALL_PREFIX=/usr/local

cmake --build ${BuildDir} --target install -- -j${cores}

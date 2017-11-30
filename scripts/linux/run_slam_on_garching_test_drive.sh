#!/bin/bash

CurDir=$(dirname $0)
ProjectDir=${CurDir}/../..
cd ${ProjectDir}

OrbSlamPlatform=`uname -m`
OrbSlamToolset=gcc.`gcc -dumpversion`
OrbSlamBuildtype=Release

BuildDir="products/cmake.make.linux.${OrbSlamPlatform}.${OrbSlamToolset}.${OrbSlamBuildtype}"

if [ "$1" = "" ]
then
    echo "Provide video path"
    exit 1
fi

if [ ! -e Vocabulary/ORBvoc.txt ]
then
    cd ${ProjectDir}/Vocabulary
    echo "Uncompress vocabulary ..."
    tar -xf ORBvoc.txt.tar.gz
    cd ..

fi

if [ ! -e Vocabulary/ORBvoc.bin ]
then
    ${BuildDir}/bin_vocabulary "Vocabulary/ORBvoc.txt" "Vocabulary/ORBvoc.bin"
fi

${ProjectDir}/${BuildDir}/mono_video \
    ${1} \
    ${ProjectDir}/Vocabulary/ORBvoc.bin \
    ${ProjectDir}/Examples/Monocular/Garching-Test-Drive.yaml

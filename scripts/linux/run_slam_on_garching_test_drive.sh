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

cd Vocabulary
if [ ! -e ORBvoc.txt ]
then
    echo "Uncompress vocabulary ..."
    tar -xf ORBvoc.txt.tar.gz
fi
cd ..

${ProjectDir}/${BuildDir}/mono_video \
    ${1} \
    ${ProjectDir}/Vocabulary/ORBvoc.txt \
    ${ProjectDir}/Examples/Monocular/Garching-Test-Drive.yaml

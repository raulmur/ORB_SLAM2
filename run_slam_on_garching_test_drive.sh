#!/bin/bash

CurDir=$(dirname $0)
OrbSlamPlatform=`uname -m`
OrbSlamToolset=gcc.`gcc -dumpversion`
OrbSlamBuildtype=Debug

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

${CurDir}/${BuildDir}/mono_video \
    ${1} \
    ${CurDir}/Vocabulary/ORBvoc.txt \
    ${CurDir}/Examples/Monocular/Garching-Test-Drive.yaml

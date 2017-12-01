#!/bin/bash

ScriptDir=$(realpath $(dirname $0))
ProjectDir=$(realpath ${ScriptDir}/../..)
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

if [ ! -e Vocabulary/ORBvoc.bin ]
then
    cd ${ProjectDir}/Vocabulary
    echo "Uncompress vocabulary ..."
    tar -xf ORBvoc.bin.tar.gz
    cd ${ProjectDir}
fi

${ProjectDir}/${BuildDir}/mono_video \
    ${1} \
    ${ProjectDir}/Vocabulary/ORBvoc.bin \
    ${ProjectDir}/Examples/Monocular/Garching-Test-Drive.yaml

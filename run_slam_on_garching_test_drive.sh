#!/bin/bash

CurDir=$(dirname $0)
OrbSlamPlatform=`uname -m`
OrbSlamToolset=gcc.`gcc -dumpversion`
BuildDir="products/cmake.make.linux.${OrbSlamPlatform}.${OrbSlamToolset}"

if [ "$1" = "" ]
then
    echo "Provide video path"
    exit 1
fi

${CurDir}/${BuildDir}/mono_video \
    ${1} \
    ${CurDir}/Vocabulary/ORBvoc.txt \
    ${CurDir}/Examples/Monocular/Garching-Test-Drive.yaml

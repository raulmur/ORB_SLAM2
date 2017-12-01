#!/bin/bash

ScriptDir=$(realpath $(dirname $0))
ProjectDir=$(realpath ${ScriptDir}/..)
cd ${ProjectDir}

if [ "$1" = "" ]
then
    echo "Provide video"
    exit 1
fi

${ProjectDir}/docker/run-from-docker.sh \
    "/root/orb-slam2/scripts/linux/run_slam_on_garching_test_drive.sh ${1}"

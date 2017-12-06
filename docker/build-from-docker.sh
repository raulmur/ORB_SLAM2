#!/bin/bash

ScriptDir=$(realpath $(dirname $0))
ProjectDir=$(realpath ${ScriptDir}/..)
cd ${ProjectDir}

docker run --rm -it --volume="$(pwd):/root/orb-slam2" \
    shanmukhananda/orb-slam2:latest \
    /bin/bash -c \
    "/root/orb-slam2/scripts/linux/build.sh"

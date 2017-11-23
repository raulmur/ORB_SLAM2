#!/bin/bash

CurDir=$(dirname $0)
ProjectDir=${CurDir}/..
cd ${ProjectDir}

sudo docker run --rm -it --volume="$(pwd):/root/orb-slam2" \
    shanmukhananda/orb-slam2:latest \
    /bin/bash -c \
    "/root/orb-slam2/scripts/linux/build.sh"

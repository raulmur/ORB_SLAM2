#!/bin/bash

CurDir=$(realpath $(dirname $0))
ProjectDir=$(realpath ${CurDir}/../..)
cd ${ProjectDir}
docker build --tag=shanmukhananda/orb-slam2:latest --file=./Dockefile .

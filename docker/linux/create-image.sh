#!/bin/bash

CurDir=$(dirname $0)
ProjectDir=${CurDir}/../..
cd ${ProjectDir}
docker build --tag=shanmukhananda/orb-slam2:latest --file=./Dockefile .

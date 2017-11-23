#!/bin/bash

CurDir=$(dirname $0)
ProjectDir=${CurDir}/..
cd ${ProjectDir}
sudo docker build --tag=shanmukhananda/orb-slam2:latest --file=./docker/Dockerfile .

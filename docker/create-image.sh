#!/bin/bash

ScriptDir=$(realpath $(dirname $0))
ProjectDir=$(realpath ${ScriptDir}/..)
cd ${ProjectDir}

docker build --tag=shanmukhananda/orb-slam2:latest --file=./docker/Dockerfile .

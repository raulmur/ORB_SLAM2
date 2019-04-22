#!/bin/bash

TAR=calc.tar.gz

URL=http://udel.edu/~nmerrill/${TAR}

DIR=.

MODEL_DIR=calc_model

mkdir -p $DIR

# skips download if already exists
wget -O ${DIR}/${TAR} -nc ${URL}

if [ ! -d "$MODEL_DIR" ]; then
	echo "Extracting the tar archive into $MODEL_DIR"
	tar -xzvf ${DIR}/${TAR}
	# Copy weights to TrainAndTest/model
	mkdir -p ../TrainAndTest/model
	cp ${MODEL_DIR}/*.caffemodel ../TrainAndTest/model
	# Copy deploy definition file to TrainAndTest/proto
	mkdir -p ../TrainAndTest/proto
	cp ${MODEL_DIR}/*.prototxt ../TrainAndTest/proto

else
	echo "Model is already downloaded. Stopping."
fi

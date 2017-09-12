#!/bin/bash

apt-get update && apt-get install -y \
    build-essential g++ autotools-dev cmake git doxygen \
    python-dev \
    libglew-dev \
    ffmpeg \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libdc1394-22-dev \
    libraw1394-dev \
    libjpeg-dev \
    libpng12-dev \
    libtiff5-dev \
    libopenexr-dev \
    libeigen3-dev \
    libopencv-dev

git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
cd ../..

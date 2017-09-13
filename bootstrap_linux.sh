#!/bin/bash

apt-get -y update
apt-get -y upgrade
apt-get -y dist-upgrade
apt-get -y autoremove

apt-get install -y \
	build-essential \
	cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
	python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev \
	libeigen3-dev

git clone https://github.com/opencv/opencv.git
cd opencv
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j $(($(nproc) + 1 ))
make install
ldconfig
cd ../..

git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir -p build
cd build
cmake ..
cmake --build .
cd ../..

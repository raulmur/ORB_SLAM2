#!/bin/bash

wget -N https://cmake.org/files/v3.9/cmake-3.9.2-Linux-x86_64.sh -P /usr/
chmod 755 /usr/cmake-3.9.2-Linux-x86_64.sh
sh /usr/cmake-3.9.2-Linux-x86_64.sh --skip-license

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
    libeigen3-dev

apt-get install -y \
	build-essential \
	cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
	python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev


git clone https://github.com/opencv/opencv.git
cd opencv
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j $(($(nproc) + 1))
make install
sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
ldconfig
apt-get update
cd ../..

git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir -p build
cd build
cmake ..
cmake --build .
cd ../..

#!/bin/bash

cmake_latest=/opt/cmake-3.9.2/bin/cmake
if [ ! -f ${cmake_latest} ] 
then 
	wget -N https://cmake.org/files/v3.9/cmake-3.9.2-Linux-x86_64.sh -P /usr/
	chmod 755 /usr/cmake-3.9.2-Linux-x86_64.sh
	mkdir -p /opt/cmake-3.9.2
	/usr/cmake-3.9.2-Linux-x86_64.sh --skip-license --prefix=/opt/cmake-3.9.2
fi


apt-get update && apt-get install -y \
    build-essential g++ autotools-dev git doxygen \
    python-dev \
    python-numpy \
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
    libgtk2.0-dev \
    pkg-config \
    libtbb2 \
    libtbb-dev \
    libpng-dev \
    libtiff-dev \
    libjasper-dev
	
if [ ! -d "/usr/local/share/OpenCV" ]
then 
	git clone https://github.com/opencv/opencv.git
	cd opencv
	mkdir -p build
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
	${cmake_latest} -j $(($(nproc) + 1))
	${cmake_latest} install
	echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf
	ldconfig
	apt-get update
	cd ../..
fi

if [ ! -d "/usr/local/lib/cmake/Pangolin" ]
then
	git clone https://github.com/stevenlovegrove/Pangolin.git
	cd Pangolin
	mkdir -p build
	cd build
	${cmake_latest} .. -DCMAKE_INSTALL_PREFIX=/usr/local
	${cmake_latest} --build . --target install
	cd ../..
fi

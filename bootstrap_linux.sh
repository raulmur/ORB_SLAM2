#!/bin/bash

if [ ! "${EUID}" = "0" ]
then
   echo "ERROR: ${0} must be run as root" 
   exit 1
fi

cores=$(nproc)
packages_dir=/root/packages
install_dir=/usr/local
mkdir -p ${packages_dir}

install_dependencies() {
    apt-get update 
    apt-get install -y \
        build-essential \
        g++ \
        autotools-dev \
        git \
        doxygen \
        python-dev \
        python-numpy \
        python3-dev \
        python3-numpy \
        libglew-dev \
        ffmpeg \
        libavcodec-dev \
        libavutil-dev \
        libavformat-dev \
        libswscale-dev \
        libdc1394-22-dev \
        libraw1394-dev \
        libjpeg-dev \
        libtiff5-dev \
        libopenexr-dev \
        libeigen3-dev \
        libgtk2.0-dev \
        pkg-config \
        libtbb2 \
        libtbb-dev \
        libpng-dev \
        libtiff-dev \
        cmake \
        apt-utils \
        wget
}

download_file() {
    cd ${packages_dir}
    folder=${packages_dir}/${1}
    url=${2}

    mkdir -p ${folder}
    cd ${folder}
    wget -N ${url}
    cd ${packages_dir}
}

download_packages() {
    opencv_url=https://github.com/opencv/opencv/archive/3.3.1.tar.gz
    pangolin_url=https://github.com/stevenlovegrove/Pangolin/archive/v0.5.tar.gz

    download_file opencv ${opencv_url}
    download_file pangolin ${pangolin_url}
}

install_opencv() {
    cd ${packages_dir}/opencv
    extracted_folder=opencv-3.3.1
    archive_file=3.3.1.tar.gz
    
    if [ ! -e ${extracted_folder} ]
    then
        mkdir -p ${extracted_folder}
        tar xzf ${archive_file} --directory=${extracted_folder} --strip-components=1
    fi
    
    cd ${extracted_folder}
    mkdir -p release
    cd release
    cmake -D CMAKE_BUILD_TYPE=release -D CMAKE_INSTALL_PREFIX=${install_dir} ..
    make -j${cores}
    make install
}

install_pangolin() {
    cd ${packages_dir}/pangolin
    extracted_folder=Pangolin-0.5
    archive_file=v0.5.tar.gz
    if [ ! -e ${extracted_folder} ]
    then
        mkdir -p ${extracted_folder}
        tar xzf ${archive_file} --directory=${extracted_folder} --strip-components=1
    fi
    cd ${extracted_folder}
    mkdir -p release
    cd release
    cmake -D CMAKE_BUILD_TYPE=release -D CMAKE_INSTALL_PREFIX=${install_dir} ..
    make -j${cores}
    make install
}

cleanup() {
    apt-get -y autoremove
    apt-get -y autoclean
    rm -rf /var/lib/apt/lists/*
}

install_dependencies
download_packages
install_opencv
install_pangolin
cleanup
#!/bin/bash
 
sudo apt-get install -y freeglut3-dev #installOpenGL
sudo apt-get install -y libglew-dev #
sudo apt-get install -y cmake #
sudo apt-get install -y libboost-dev libboost-thread-dev libboost-filesystem-dev #
sudo apt-get install -y libpython2.7-dev python-numpy
sudo apt-get install -y g++
sudo apt-get install -y libopencv-dev
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y libblas-dev
sudo apt-get install -y liblapack-dev
sudo apt-get install -y git
#
#opencv:libopencv-dev 开发包
#eigen:libeigen3-dev
#blas:libblas-dev
#lapack:liblapack-dev
#
cd ~/DeveloperTool/ #come to the  default file 
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake -DCPP11_NO_BOOST=1 ..
make -j
#install the slam page from github
cd ~/DeveloperTool/ #come to the default file
 git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
cd ORB_SLAM2
chmod +x build.sh
./build.sh

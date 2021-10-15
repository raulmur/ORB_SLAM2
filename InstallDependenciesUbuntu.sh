#!/bin/bash
sudo apt-get install g++ cmake libeigen3-dev libopencv-dev libglew-dev && \
cd .. && git clone https://github.com/stevenlovegrove/Pangolin.git && \
pushd Pangolin && \
    mkdir build && \
    pushd build && \
        cmake .. && make -j`nproc` && \
    popd && \
popd

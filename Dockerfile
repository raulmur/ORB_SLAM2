# docker image to run ORB-SLAM2
FROM ubuntu:18.04
MAINTAINER devin@monodrive.io

# setup
ENV BASE_DIR /home/vo
ENV DEBIAN_FRONTEND noninteractive
WORKDIR ${BASE_DIR}
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    git \
    wget

# c++
RUN apt-get install -y \
    build-essential \
    gcc \
    cmake

# pangolin
RUN apt-get install -y \
    libgl1-mesa-dev \
    libglew-dev

RUN git clone https://github.com/stevenlovegrove/Pangolin.git ${BASE_DIR}/pangolin
WORKDIR ${BASE_DIR}/pangolin/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN cmake --build . -- -j4

# opencv
RUN apt-get install -y \
    pkg-config \
    libgtk2.0-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev

RUN git clone https://github.com/opencv/opencv.git --branch 3.2.0 --depth 1 ${BASE_DIR}/opencv
WORKDIR ${BASE_DIR}/opencv/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j4
RUN make install

# eigen
WORKDIR ${BASE_DIR}/dump
RUN wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz
RUN tar -xzf 3.3.7.tar.gz
RUN mv eigen-eigen-323c052e1731 /usr/local/include/eigen

# python
RUN apt-get install -y python3-pip
RUN pip3 install PyYAML

# orb-slam2
COPY Thirdparty ${BASE_DIR}/orbslam2/Thirdparty
COPY Vocabulary ${BASE_DIR}/orbslam2/Vocabulary
COPY cmake_modules ${BASE_DIR}/orbslam2/cmake_modules
COPY include ${BASE_DIR}/orbslam2/include

COPY CMakeLists.txt ${BASE_DIR}/orbslam2/CMakeLists.txt

# orb-slam2 // dbow2
WORKDIR ${BASE_DIR}/orbslam2/Thirdparty/DBoW2/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j4

# orb-slam2 // g2o
WORKDIR ${BASE_DIR}/orbslam2/Thirdparty/g2o/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j4

# orb-slam2 // vocab
WORKDIR ${BASE_DIR}/orbslam2/Vocabulary
RUN tar -xf ORBvoc.txt.tar.gz

# orb-slam // source
COPY src ${BASE_DIR}/orbslam2/src
COPY Examples ${BASE_DIR}/orbslam2/Examples

# orb-slam2 // build
WORKDIR ${BASE_DIR}/orbslam2/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j4

# orb-slam2 // python
COPY python ${BASE_DIR}/orbslam2/python

# runtime
WORKDIR ${BASE_DIR}
CMD ./orbslam2/Examples/Monocular/mono_kitti orbslam2/Vocabulary/ORBvoc.txt etc/monoDrive.yaml workspace/kitti/dataset/sequences/00 workspace/orbslam2/keyframes.txt

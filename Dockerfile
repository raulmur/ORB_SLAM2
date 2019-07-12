# docker image to run ORB-SLAM2
FROM ubuntu:18.04
MAINTAINER devin@monodrive.io

# setup
ENV BASE_DIR /home/vo
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

# opencv
RUN apt-get install -y \
    pkg-config \
    libgtk2.0-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev

RUN git clone https://github.com/opencv/opencv.git --branch 3.2.0 --depth 1 ${BASE_DIR}/opencv
WORKDIR ${BASE_DIR}/opencv/build
RUN cmake ..
RUN make -j7
RUN make install

# eigen
WORKDIR ${BASE_DIR}/dump
RUN wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz
RUN tar -xzf 3.3.7.tar.gz
RUN mv eigen-eigen-323c052e1731 /usr/local/include/eigen

# runtime
WORKDIR ${BASE_DIR}
CMD ["ls"]
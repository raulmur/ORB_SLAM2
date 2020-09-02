# docker image for ORB-SLAM2 build
FROM ubuntu:18.04 as build

# setup
ENV BASE /home/orbslam2
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR ${BASE}

# setup pre-reqs
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    git \
    wget \
    build-essential \
    gcc \
    cmake

# pangolin
RUN apt-get install -y \
    libgl1-mesa-dev \
    libglew-dev

RUN git clone https://github.com/stevenlovegrove/Pangolin.git ${BASE}/pangolin
WORKDIR ${BASE}/pangolin/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN cmake --build . -- -j4

# opencv pre-reqs
RUN apt-get install -y \
    libgtk2.0-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev

# opencv build
RUN release="3.2.0" && \
    mkdir -p ${BASE}/dump && \
    git clone https://github.com/opencv/opencv.git ${BASE}/dump/opencv && \
    git -C ${BASE}/dump/opencv checkout tags/${release} && \
    mkdir -p ${BASE}/dump/opencv_build && \
    mkdir -p ${BASE}/dump/opencv_install && \
    cd ${BASE}/dump/opencv_build && \
    cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=${BASE}/dump/opencv_install \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_OPENCV_PYTHON2=OFF \
    -D BUILD_OPENCV_PYTHON3=OFF \
    -D ENABLE_PRECOMPILED_HEADERS=OFF \
    ../opencv && \
    make -j 4 && \
    make install

ENV PATH ${BASE}/dump/opencv_install:${PATH}

# eigen
WORKDIR ${BASE}/dump
RUN wget -O 3.3.7.tar.gz https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
RUN tar -xzf 3.3.7.tar.gz
RUN mv eigen-3.3.7 /usr/local/include/eigen

# orb-slam2 // resources
COPY Thirdparty ${BASE}/orbslam2/Thirdparty
COPY Vocabulary ${BASE}/orbslam2/Vocabulary
COPY cmake_modules ${BASE}/orbslam2/cmake_modules
COPY include ${BASE}/orbslam2/include
COPY CMakeLists.txt ${BASE}/orbslam2/CMakeLists.txt

# orb-slam2 // dbow2
WORKDIR ${BASE}/orbslam2/Thirdparty/DBoW2/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j4

# orb-slam2 // g2o
WORKDIR ${BASE}/orbslam2/Thirdparty/g2o/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j4

# orb-slam2 // vocab
WORKDIR ${BASE}/orbslam2/Vocabulary
RUN tar -xf ORBvoc.txt.tar.gz

# orb-slam // source
COPY src ${BASE}/orbslam2/src
COPY Examples ${BASE}/orbslam2/Examples

# orb-slam2 // build
WORKDIR ${BASE}/orbslam2/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j4


# runtime image for ORB-SLAM2
FROM ubuntu:18.04

# setup
ENV BASE /home/orbslam2
ARG DEBIAN_FRONTEND=noninteractive
RUN useradd --create-home --home-dir ${BASE} --uid 1000 --shell /bin/bash orbslam2

# install lib dependencies
RUN apt-get update && \
    apt-get install -y \
    libgtk2.0-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgl1-mesa-dev \
    libglew-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# pip packages
RUN pip3 install PyYAML

# copy over opencv installation
COPY --from=build ${BASE}/dump/opencv_install/bin/ /usr/local/bin/
COPY --from=build ${BASE}/dump/opencv_install/share/ /usr/local/share/
COPY --from=build ${BASE}/dump/opencv_install/include/ /usr/local/include/
COPY --from=build ${BASE}/dump/opencv_install/lib/ /usr/local/lib/

# copy over built libraries
COPY --from=build /usr/local/include/eigen /usr/local/include/eigen
COPY --from=build ${BASE}/pangolin/build/src/libpangolin.so /usr/local/lib/
COPY --from=build ${BASE}/orbslam2/Thirdparty/DBoW2/lib/libDBoW2.so /usr/local/lib/
COPY --from=build ${BASE}/orbslam2/Thirdparty/g2o/lib/libg2o.so /usr/local/lib/
COPY --from=build ${BASE}/orbslam2/lib/libORB_SLAM2.so /usr/local/lib/

# link
RUN ldconfig

# copy over built orbslam2 executable and other runtime dependencies
COPY --from=build ${BASE}/orbslam2/Examples/Monocular/mono_kitti ${BASE}/mono_kitti
COPY --from=build ${BASE}/orbslam2/Vocabulary/ORBvoc.txt ${BASE}/ORBvoc.txt
COPY python/generate_params_file.py ${BASE}/generate_params_file.py

# runtime
USER orbslam2
WORKDIR ${BASE}
CMD ./mono_kitti ORBvoc.txt etc/monoDrive.yaml workspace/kitti/dataset/sequences/00 workspace/orbslam2/keyframes.txt

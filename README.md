# ORB-SLAM2 with ByteTrack + YOLOv5 and Jonint Optimization of Object and Camera Pose
Author: YM Chang


# Prerequisites

Tested with:
- Ubuntu 18.04 Docker container in Windows Subsystem for Linux 2 (WSL2)
- CUDA 11.7 and cuDNN 8.9.0
- OpenCV 3.4.16
- onnxruntime 1.11.0 with GPU support
- YOLOv5 6.2 pre-trained model (trained on COCO 2017 dataset)

## Pangolin
See [Pangolin](https://github.com/stevenlovegrove/Pangolin) for how to build Pangolin.

## OpenCV
We used OpenCV 3.4.16. See [OpenCV](http://opencv.org).
Notice that DNN module was introduced from **OpenCV 3.4.0.**, which is required for onnx model loading.

## Eigen3
Required for g2o and ByteTrack.
See [Eigen](http://eigen.tuxfamily.org) for how to build Eigen3.

## DBoW2 and g2o (Included in Thirdparty folder)
The original repository of [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) uses modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the Thirdparty folder.

## CUDA and cuDNN
See [CUDA](https://developer.nvidia.com/cuda-downloads) for installation of CUDA.

See [cuDNN](https://developer.nvidia.com/cudnn) for installation of cuDNN.

## onnxruntime
Here we have included the onnxruntime 1.11.0 library in the ByteTrack folder, which supports GPU acceleration.
See [onnxruntime](https://github.com/microsoft/onnxruntime) for more information and releases.


# Build

1. Build the thirdparty libraries
```
cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j10

cd ../../g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j10
```

2. Build ByteTrack
```
cd ../../../ByteTrack
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j10
```

3. Build ORB-SLAM2
```
cd ../../
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j10
```

# Run

# Citation
- ORB-SLAM2: [ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras](https://arxiv.org/abs/1610.06475)
- ByteTrack: [ByteTrack: Multi-Object Tracking by Associating Every Detection Box](https://arxiv.org/abs/2107.06278)
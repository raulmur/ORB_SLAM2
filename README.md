# Add Notes for ORB-SLAM2 and fix some bugs
> Reference:
> - https://github.com/raulmur/ORB_SLAM2
> - https://github.com/zhaoxuhui/ORB-Code-Notes


|  依赖库  |    说明    | 作用                         |
| :------: | :--------: | ---------------------------- |
| OpenCV3  | 图像处理库 | 图像灰度转换、FAST特征点提取 |
|  DBoW2   |   词袋库   | 特征描述向量                 |
|   g2o    |   优化库   | 位姿优化、BA优化             |
|  Eigen3  | 矩阵运算库 | 为g2o提供支持                |
| Pangolin |  可视化库  | ...                          |


### 单目情况处理流程(Monocular case processing flow)
```
1. 读取图像和时间戳 -> Mono_kitti::LoadImages(), System::TrackMonocular
2. 将图像转换为灰度图 -> Tracking::GrabImageMonocular()
3. 用灰度图初始化Frame: -> Frame::Frame()
    3.1 提取特征点 -> Frame::ExtractORB()
        3.1.1 重载括号操作符，提取左目(右目)特征点 -> ORBextractor::operator() 
            3.1.1.1 计算图像金字塔每层的图像的大小 -> ORBextractor::ComputePyramid()
            3.1.1.2 网格化金字塔的每层图像，并利用FAST方法提取每个网格中的图像 -> ORBextractor::ComputeKeyPointsOctTree()
            3.1.1.3 计算特征点的描述子 -> ORBextractor::computeDescriptors()
    3.2 根据畸变参数对特征点坐标进行校正 -> Frame::UndistortKeyPoints()
    3.3 将校正后的特征点分配到网格 -> Frame::AssignFeaturesToGrid()
4. 跟踪 -> Tracking::Track()
    4.1 单目初始化 -> Tracking::MonocularInitialization()
        4.1.1 初始化(相机内参、关键点、尺度因子、RANSAC最大迭代次数) -> Initializer::Initialize()
        4.1.2 第一帧与第二帧特征点匹配 -> ORBmatcher::ORBmatcher()
        4.1.3 第一帧与第二帧初始化 -> Initializer::Initializer()
            4.1.3.1 寻找单应矩阵 -> Initializer::FindHomography()
                4.1.3.1.1 特征点坐标归一化 -> Initializer::Normalize()
                4.1.3.1.2 计算单应矩阵 -> Initializer::ComputeH21()
                4.1.3.1.3 计算单应矩阵得分 -> Initializer::CheckHomography()
            4.1.3.2 寻找本质矩阵 -> Initializer::FindFundamental()
                4.1.3.2.1 特征点坐标归一化 -> Initializer::Normalize()
                4.1.3.2.2 计算本质矩阵 -> Initializer::ComputeF21()
                4.1.3.2.3 计算本质矩阵得分 -> Initializer::CheckFundamental()
            4.1.3.3 分解单应矩阵 -> Initializer::ReconstructH()
				4.1.3.3.1 SVD分解 -> cv::SVD::compute()
				4.1.3.3.2 检验R, t -> Initializer::CheckRT()
					4.1.3.3.2.1 计算深度 -> Initializer::Triangulate()
            4.1.3.4 分解本质矩阵 -> Initializer::ReconstructF()
				4.1.3.4.1 恢复四个运动假设 -> Initializer::DecomposeE()
                4.1.3.4.2 检验R, t -> Initializer::CheckRT()
					4.1.3.4.2.1 计算深度 -> Initializer::Triangulate()
        4.1.4 创建单目初始化地图 -> Tracking::CreateInitialMapMonocular()
			4.1.4.1 初始化两帧为关键帧 -> KeyFrame::KeyFrame()
				4.1.4.1.1 设置位姿 -> KeyFrame::SetPose()
			4.1.4.2 计算初始化两帧描述子的BoW -> KeyFrame::ComputeBoW()
				4.1.4.2.1 将描述子由Mat转换为vector -> Converter::toDescriptorVector()
				4.1.4.2.2 调用DBoW2的API计算特征点描述子对应的BoW向量 -> mpORBvocabulary->transform()
			4.1.4.3 向地图中添加关键帧 -> Map::AddKeyFrame()
			4.1.4.4 初始化地图点 -> MapPoint::MapPoint()
			4.1.4.5 向两个关键帧中添加地图点 -> KeyFrame::AddMapPoint()
			4.1.4.6 向地图中添加两个关键帧和对应地图点的观测 -> MapPoint::AddObservation()
			4.1.4.7 同一个地图点存在于多个关键帧中，计算最具代表性的描述子 -> MapPoint::ComputeDistinctiveDescription()
			4.1.4.8 更新深度信息 -> MapPoint::UpdateNormalAndDepth()
			4.1.4.9 将地图点添加到地图中 -> Map::AddMapPoint()
			4.1.4.10 更新初始化两帧之间的连接 -> KeyFrame::UpdateConnections()
			4.1.4.11 对地图点、关联的关键帧以及观测进行全局BA优化 -> Optimizer::GlobalBundleAdjustment()
			4.1.4.12 计算初始化两帧的平均深度 -> ComputeSceneMedianDepth()
			4.1.4.13 初始基线尺度调整 -> KeyFrame::GetPose(), KeyFrame::SetPose()
			4.1.4.14 根据调整后的初始化第二帧的位姿，缩放地图点 -> KeyFrame::GetMapPointMatches(), MapPoint::GetWorldPos(), MapPoint::SetWorldPos()
			4.1.4.15 将初始化的两个关键帧插入mpLocalMapper -> LocalMapping::InsertKeyFrame()
			4.1.4.16 ...
	4.2 更新地图点在当前帧中是否可以被观测到 -> FrameDrawer::update()
		4.2.1 检查地图点是否在当前帧中可以被观测到 -> MapPoint::Ovservations()
    4.3 检查上一帧中跟踪的某些mappoints是否被更新 -> Tracking::CheckReplacedInLastFrame()
		4.3.1 获取当前地图点是否被替换 -> MapPoint::GetReplaced()
    4.4 根据参考关键帧跟踪 -> Tracking::TrackReferenceKeyFrame()
		4.4.1 计算当前帧的词袋向量 -> Frame::ComputeBoW()
			4.4.1.1 将当前帧的描述子由Mat按行拆分转换为vector -> Converter::toDescriptorVector()
			4.4.1.2 将当前帧的描述子由vector转换为BoW2::BowVector和BoW2::FeatureVector -> ORB::ORBVocabulary::transform()(type: DBoW2, template)
		4.4.2 初始化matcher -> ORBmatcher::ORBmatcher()
		4.4.3 根据词袋方法，寻找参考关键帧(KeyFrame)和当前帧(Frame)的匹配点 -> ORBmatcher::SearchByBoW()
			4.4.3.1 获取参考关键帧对应的地图点 -> KeyFrame::GetMapPointMatches()
			4.4.3.2 获取参考关键帧的特征描述向量 -> KeyFrame::mFeatVec(type: DBoW2::FeatureVector)
			4.4.3.3 对属于同一词汇节点的的ORB执行匹配 -> ...
				4.4.3.3.1 将参考关键帧和当前帧的ORB词袋对齐到同一层
				4.4.3.3.2 以参考关键帧中的地图点为基准
				4.4.3.3.3 遍历参考关键帧中的地图点，计算该点的描述子与当前帧中所有地图点的描述子两两之间的距离 -> ORBmatcher::DescriptorDistance()
				4.4.3.3.4 保留最佳匹配(描述子之间的距离最小，且距离小于TH_LOW)，计算该匹配的方向梯度直方图
			4.4.3.4 找到方向梯度直方图中数量最多的三个bin，保留这些bin对应的的特征点匹配对 -> ORBmatcher::ComputeThreeMaxima()
		4.4.4 (设置当前帧的地图点)，设置当前帧的位姿 -> Frame::SetPose()
			4.4.4.1 更新变换矩阵mTcw
			4.4.4.2 根据mTcw更新mRcw, mRwc, mtcw, mOw -> Frame::UpdatePoseMatrices()
		4.4.5 优化当前帧的位姿 -> Optimizer::PoseOptimization()
			4.4.5.1 构造g2o优化器
			4.4.5.2 ...
		4.4.6 检查所有地图点，删除外点，查询观测>0的点 -> MapPoint::Observation()
	4.5 根据运动模型跟踪 -> Tracking::TrackWithMotionModel()
		4.5.1 初始化matcher -> ORBmatcher::ORBmatcher()
		4.5.2 根据参考关键帧更新上一帧的位姿 -> Tracking::UpdateLastFrame()
		4.5.3 根据运动量和上一帧的变换矩阵设置当前帧的位姿 -> Frame::SetPose()
		4.5.4 根据投影方法，寻找上一帧(Frame)与当前帧(Frame)的匹配点 -> ORBmatcher::SearchByProject()
		4.5.5 使用全部的匹配优化当前帧的位姿 -> Optimizer::PoseOptimizer()
		4.5.6 检查所有地图点，删除外点，查询观测>0的点 -> MapPoint::Observation()
    4.6 重定位 -> Tracking::Relocalization()
```



# ORB-SLAM2
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**13 Jan 2017**: OpenCV 3 and Eigen 3.3 are now supported.

**22 Dec 2016**: Added AR demo (see section 7).

ORB-SLAM2 is a real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. We provide examples to run the SLAM system in the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) as stereo or monocular, in the [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset) as RGB-D or monocular, and in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as stereo or monocular. We also provide a ROS node to process live monocular, stereo or RGB-D streams. **The library can be compiled without ROS**. ORB-SLAM2 provides a GUI to change between a *SLAM Mode* and *Localization Mode*, see section 9 of this document.

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>


### Related Publications:

[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORB-SLAM2 is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM2 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM2 (Monocular) in an academic work, please cite:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

if you use ORB-SLAM2 (Stereo or RGB-D) in an academic work, please cite:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 12.04**, **14.04** and **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## ROS (optional)
We provide some examples to process the live input of a monocular, stereo or RGB-D camera using [ROS](ros.org). Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed.

# 3. Building ORB-SLAM2 library and examples

Clone the repository:
```
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.

# 4. Monocular Examples

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```

## KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

# 5. Stereo Examples

## KITTI Dataset

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/mav0/cam0/data PATH_TO_SEQUENCE/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```

# 6. RGB-D Example

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# 7. ROS Examples

### Building the nodes for mono, monoAR, stereo and RGB-D
1. Add the path including *Examples/ROS/ORB_SLAM2* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM2:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM2/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Running Monocular Augmented Reality Demo
This is a demo of augmented reality where you can use an interface to insert virtual cubes in planar regions of the scene.
The node reads images from topic `/camera/image_raw`.

  ```
  rosrun ORB_SLAM2 MonoAR PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Running Stereo Node
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM2/Stereo. You will need to provide the vocabulary file and a settings file. If you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**.

  ```
  rosrun ORB_SLAM2 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```
  
**Example**: Download a rosbag (e.g. V1_01_easy.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab:
  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
  ```
  
Once ORB-SLAM2 has loaded the vocabulary, press space in the rosbag tab. Enjoy!. Note: a powerful computer is required to run the most exigent sequences of this dataset.

### Running RGB_D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM2/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
# 8. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular, stereo and RGB-D cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the ORB-SLAM2 library and how to pass images to the SLAM system. Stereo input must be synchronized and rectified. RGB-D input must be synchronized and depth registered.

# 9. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 


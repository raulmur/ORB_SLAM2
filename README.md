# muse-gr/ORB-SLAM2
*これはpublicレポジトリです。秘密情報を入れないように！*  
*This is a public repository. Do not push any confidential information!!*

ORB-SLAM2 modified for development

# Build
See the original repo: https://github.com/raulmur/ORB_SLAM2

# Run
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/
export LIBGL_ALWAYS_INDIRECT=1
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/path/to/ORB_SLAM2/Examples/ROS

rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt muse/muse.yaml false
```

# ROS topics
input:  
- */camera/left/image_raw* : left image (rectified, undistorted)
- */camera/right/image_raw* : right image (rectified, undistorted)

output:  
- */cube/data/vslam_localization/pose* : Estimated pose  


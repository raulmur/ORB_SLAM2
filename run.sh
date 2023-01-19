export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/
export LIBGL_ALWAYS_INDIRECT=1
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/tomo/work/temp/ORB_SLAM2/Examples/ROS

rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt muse/muse.yaml false

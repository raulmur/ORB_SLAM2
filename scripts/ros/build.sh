echo "Building ROS nodes"

CurDir=$(dirname $0)
ProjectDir=${CurDir}/../..
cd ${ProjectDir}

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j

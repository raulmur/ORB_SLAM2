echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
#CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:./pango-lib/lib/cmake
cmake .. -DROS_BUILD_TYPE=Release
make -j

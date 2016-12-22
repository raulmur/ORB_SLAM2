echo "Configuring and building Thirdparty/DBoW2 ..."

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j

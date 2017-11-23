echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j4


echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building Thirdparty/DBoW2 Thirdparty/g2o ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build ./Thirdparty/DBoW2/ --config Release
cmake --build ./Thirdparty/g2o/ --config Release
cmake --build .

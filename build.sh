echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
rm -rf build/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

rm -rf build/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

rm -rf build/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make 

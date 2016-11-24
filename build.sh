echo "Configuring and building Thirdparty/DBoW2 ..."

RELEASE_TYPE=RelWithDebInfo


cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=${RELEASE_TYPE}
make clean
make -j1

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=${RELEASE_TYPE}
make clean
make -j1

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=${RELEASE_TYPE}
make clean
make -j1

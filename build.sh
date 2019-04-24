echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

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

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCaffe_ROOT_DIR=${HOME}/eecs568-project/caffe -DCPU_ONLY=1 ..
#cmake -DCMAKE_BUILD_TYPE=Release -DCaffe_ROOT_DIR=${HOME}/caffe -DCPU_ONLY=1 ..

make -j${nproc}

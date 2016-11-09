echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make clean
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make clean
make -j

cd ../../../


echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make clean
make -j

cd ..

./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI04-12-BVANDEPO.yaml /home/bvandepo/orbslam/dataset/sequences/16
#hexdump  Vocabulary/ORBvoc.txt.bin  -v -e '/1 "%01u\n"' >Vocabulary/Deci.txt


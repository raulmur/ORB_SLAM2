
echo "Configuring and building ORB_SLAM2 ..."
cd build
make -j
cd ..

#./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI04-12-BVANDEPO.yaml /home/bvandepo/orbslam/dataset/sequences/16
#hexdump  Vocabulary/ORBvoc.txt.bin  -v -e '/1 "%01u\n"' >Vocabulary/Deci.txt


echo "Installing Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2/build
make install

echo "Installing Thirdparty/g2o ..."
mkdir build
cd ../../g2o/build
make install

echo "Configuring and building ORB_SLAM2 ..."
cd ../../../build
make install

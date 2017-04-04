
src_root=./src
orbslampy_root=$src_root/orbslampy

swig_interface_root=$orbslampy_root/swig
swig_files=$swig_interface_root/orbslampy.i
swig_output_dir=$orbslampy_root/src

cmake_args=4

# run swig
# swig -Wall -python -I/usr/local/include/ -c++ -debug-classes $swig_files

# need to copy files to theyre respective places
# mv $swig_interface_root/orbslampy_wrap.cxx $swig_output_dir

# create devel directory
mkdir -p devel/cmake/modules
mkdir -p devel/include
mkdir -p devel/lib
mkdir -p devel/bin

protoc -I=. --python_out=./devel ./messages.proto

# create build directory
mkdir build
cd build

# build ORBSLAM2
# cmake ../ORB_SLAM2
# make

# run cmake
cmake .. # $cmake_args
make
cd ..

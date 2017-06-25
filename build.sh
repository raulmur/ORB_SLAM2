cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

rm -rf build/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make 

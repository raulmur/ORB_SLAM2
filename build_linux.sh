echo "Uncompress vocabulary ..."

OrbSlamPlatform=`uname -m`
OrbSlamToolset=gcc.`gcc -dumpversion`
Buildtype=Release

if [ ! -z "$1" ] 
then
    OrbSlamPlatform="$1"
fi

if [ ! -z "$2" ] 
then
    OrbSlamToolset="$2"
fi

if [ ! -z "$3" ] 
then
    Buildtype="$3"
fi

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building Thirdparty/DBoW2 Thirdparty/g2o ORB_SLAM2 ..."

BuildDir=products/linux-${OrbSlamPlatform}-${OrbSlamToolset}-${Buildtype}
if [ ! -e ${BuildDir} ] 
then 
	mkdir -p "${BuildDir}"
fi

cd ${BuildDir}

cmake ../.. -DCMAKE_BUILD_TYPE=Release -DBUILD_THIRDPARTY_LIB=ON
cmake --build ./Thirdparty/DBoW2/ --config Release
cmake --build ./Thirdparty/g2o/ --config Release
cmake --build .

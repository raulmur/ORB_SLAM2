What is Pangolin {#mainpage}
====================================

Pangolin is a lightweight portable rapid development library for managing OpenGL
display / interaction and abstracting video input. At its heart is a simple
OpenGl viewport manager which can help to modularise 3D visualisation without
adding to its complexity, and offers an advanced but intuitive 3D navigation
handler. Pangolin also provides a mechanism for manipulating program variables
through config files and ui integration, and has a flexible real-time plotter
for visualising graphical data.

The ethos of Pangolin is to reduce the boilerplate code that normally
gets written to visualise and interact with (typically image and 3D
based) systems, without compromising performance. It also enables write-once
code for a number of platforms, currently including Windows, Linux, OSX, Android
and IOS.

## Code ##

Find the latest version on [Github](http://github.com/stevenlovegrove/Pangolin):

```
git clone https://github.com/stevenlovegrove/Pangolin.git
```

Current build status on [Drone.io](https://drone.io/github.com/stevenlovegrove/Pangolin)
![Build Status](https://drone.io/github.com/stevenlovegrove/Pangolin/status.png)

## Dependencies ##

Optional dependencies are enabled when found, otherwise they are silently disabled.
Check the CMake configure output for details.

### Required Dependencies ###

* OpenGL (Desktop / ES / ES2)

* Glew
 * (win) built automatically
 * (deb) sudo apt-get install libglew-dev
 * (mac) sudo port install glew

* CMake (for build environment)
 * (win) http://www.cmake.org/cmake/resources/software.html
 * (deb) sudo apt-get install cmake
 * (mac) sudo port install cmake

### Recommended Dependencies ###

* Boost (optional with C++11. Configure with 'cmake -DCPP11_NO_BOOST=1 ..' )
 * (win) http://www.boost.org/users/download/
 * (deb) sudo apt-get install libboost-dev libboost-thread-dev libboost-filesystem-dev
 * (mac) sudo port install boost

* Python2 / Python3, for drop-down interactive console
 * (win) http://www.python.org/downloads/windows
 * (deb) sudo apt-get install libpython2.7-dev
 * (mac) preinstalled with osx

### Optional Dependencies for video input ###

* FFMPEG (For video decoding and image rescaling)
 * (deb) sudo apt-get install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev

* DC1394 (For firewire input)
 * (deb) sudo apt-get install libdc1394-22-dev libraw1394-dev

* libuvc (For cross-platform webcam video input via libusb)
 * git://github.com/ktossell/libuvc.git

* libjpeg, libpng, libtiff, libopenexr (For reading still-image sequences)
 * (deb) sudo apt-get install libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev

* OpenNI / OpenNI2 (For Kinect / Xtrion / Primesense capture)

* DepthSense SDK

### Very Optional Dependencies ###

* Eigen / TooN (These matrix types supported in the Pangolin API.)

* CUDA Toolkit >= 3.2 (Some CUDA header-only interop utilities included)
 * http://developer.nvidia.com/cuda-downloads

* Doxygen for generating html / pdf documentation.

## Building ##

Pangolin uses the CMake portable pre-build tool. To checkout and build pangolin in the
directory 'build', enabling C++11 support instead of using Boost, execute the
following at a shell (or the equivelent using a GUI):

```
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake -DCPP11_NO_BOOST=1 ..
make -j
```

If you would like to build the documentation and you have Doxygen installed, you
can execute:

```
make doc
```

## Issues ##

Please visit [Github Issues](https://github.com/stevenlovegrove/Pangolin/issues) to view and report problems with Pangolin. Issues and pull requests should be raised against the devel branch which contains the current development version.

Please note; most Pangolin dependencies are optional - to disable a dependency which may be causing trouble on your machine, simply blank out it's include and library directories with a cmake configuration tool (e.g. ccmake or cmake-gui).


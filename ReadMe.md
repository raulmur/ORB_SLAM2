orbslampy is a python module that exposes ORB_SLAM2 to the python language. It (in progress) uses numpy for matrix representations.

To build orbslampy, you will need the following dependencies:

cmake (an independent c/c++ build language)
swig (a tool to autogenerate interface code between c/c++ and a target language)
numpy
opencv-python

Please read the ReadMe in /src/ORB_SLAM2 for additional dependencies imposed by the ORB_SLAM2 package.

To build orbslampy, in this directory, open a shell and type "./build.bash"
The binaries will be in the devel/ directory

Something to note is that c++ std::vectors retain their member function names in Python.
So, calling len(<...>) will have to be replaced with <...>.size() and so forth.

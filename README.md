# ORB-SLAM2
https://github.com/raulmur/ORB_SLAM2

# This project
Add some mechanisms to make ORB_SLAM2 initializes or transform its coordinate to the coordinate recognized by a marker detector. 

# How to use
Inherit MarkerDetector class and implemention detect() function. 
Pass the marker detector to the system as the last parameter while initializing. 
The marker detector runs in an extra thread.
Note that by default, the marker detector is switched off. It works as the original ORB_SLAM2.

# Initialize from marker
When the map was not created, system will try to initialize from a marker-contained frame.
Markers must provide enough keypoints inside to create the initial keyframe.
To prevent some problems cause by there is only one keyframe after initializing, te second keyframe must have a minium distance from the first.

# Transform coordinates to the marker
Whe the map was already created, system will try to transform it's coordinate to the marker's.
The camera pose of each keyframe and the position of each keypoint will be transformed and rescale.
This currently works only once.

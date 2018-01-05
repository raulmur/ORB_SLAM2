/**
* This file is part of the odometry integration of pepper in SLAM
* Author: Tim Resink
* Email: timresink@gmail.com
*/

// ROS
#include <ros/ros.h>

// TF
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Eigen
#include <Eigen/Geometry>


// own files
#include "SubscribeHandler.hpp"

// g2o
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

// using namespace Eigen;



int main(int argc, char * argv[]){
	// initialize ROS. Allows for name remapping (something:/= "somethingelse")
	// in the command line.
    // Listener is the name of the node written here
    ros::init(argc, argv, "MaquiOdom");

    // NodeHandle is the main acces point to communications with the ROS system.
    ros::NodeHandle NodeHandler;

    // buffer for the transform messages
    tf::TransformListener TfListener;
    SubscribeHandler maquiHandler(argv[1], argv[2], &NodeHandler, &TfListener);

    if(argc != 3)
    {
         cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
         maquiHandler.Shutdown();
        return 1;
    }





	ros::spin();

	maquiHandler.Shutdown();

	return 0;
};


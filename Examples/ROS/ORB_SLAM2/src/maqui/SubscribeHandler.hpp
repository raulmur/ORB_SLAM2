/**
* Author: Tim Resink
* Email: timresink@gmail.com


TODOs:
- Add settings to settings file
    -     cameraTopic
    -     tfTopic



*/

// STD
#include <iostream>
//#include <string>

// ROS
#include<ros/ros.h>
#include <sensor_msgs/Image.h>

// TF
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>

// cv bridge
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

//Eigen
#include <Eigen/Geometry>

// ORB-SLAM
#include "System.h"


class SubscribeHandler{
public:
	// constructors
    SubscribeHandler(const string &strVocFile,
                     const string &strSettingsFile, ros::NodeHandle *pNodeHandler,
                     tf::TransformListener *pTFlistener);

    // variables
    tf::StampedTransform tfT_w_c;
    cv::Mat cvT_w_c;
    cv_bridge::CvImageConstPtr cv_ptr;

	// methods
	void Shutdown();

private:
    // methods
    void GrabImage(const sensor_msgs::ImageConstPtr &msg);
    cv::Mat tfToMat(const tf::StampedTransform& tfT, const float scale);

    // ROS
    ros::Subscriber subImage;

    ros::NodeHandle* mpNodeHandler;
    tf::TransformListener* mpTFlistener;

    // ORB SLAM pointer
    ORB_SLAM2::System* mpSLAM;

    //topics
    std::string cameraTopic;
    std::string tfTopic;
    std::string cameraFrameTopic;
    std::string worldFrameTopic;

    // flags
    bool mbReferenceWorldFrame;
    const float scale = 1.f;
};

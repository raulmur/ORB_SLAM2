/**
* This file is part of the odometry integration of pepper in SLAM
* Author: Tim Resink
* Email: timresink@gmail.com
*/

#include"SubscribeHandler.hpp"
using namespace Eigen;



SubscribeHandler::SubscribeHandler(const string &strVocFile,
                                   const string &strSettingsFile, ros::NodeHandle* pNodeHandler,
                                   tf::TransformListener* pTFlistener):
mpNodeHandler(pNodeHandler),
mpTFlistener(pTFlistener),
mbReferenceWorldFrame(false)
{
    // Initialize TODO put in setting file
        // Topics to subscribe
//    cameraTopic      = "/pepper_robot/camera/bottom/image_raw";
//    cameraFrameTopic = "/CameraBottom_optical_frame";
//    worldFrameTopic  = "/map";
//    tfTopic          = "tf";
      cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
      cameraTopic = (std::string) fsSettings["Topic.Camera"];
      cameraFrameTopic = (std::string) fsSettings["Topic.CameraFrame"];
      worldFrameTopic = (std::string) fsSettings["Topic.WorldFrame"];
      tfTopic =(std::string) fsSettings["Topic.TF"];

    mpSLAM = new ORB_SLAM2::System(strVocFile, strSettingsFile, ORB_SLAM2::System::ODOMETRY,true);

    subImage = mpNodeHandler->subscribe(cameraTopic, 1, &SubscribeHandler::GrabImage, this);


   // Initialize ORB system
   // argument 4 boolean is user viewer
}



void SubscribeHandler::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    try
    {
        mpTFlistener->waitForTransform(cameraFrameTopic,worldFrameTopic, ros::Time(0), ros::Duration(0.0001));
        mpTFlistener->lookupTransform(cameraFrameTopic, worldFrameTopic, ros::Time(0), tfT_w_c);

    }
    catch(tf::TransformException& e)
    {
        ROS_WARN("TF exception while grabbing camera transform \n %s", e.what());
    }

    cvT_w_c = tfToMat(tfT_w_c, scale);
    mpSLAM->SetOdomPose(cvT_w_c);
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

}





cv::Mat SubscribeHandler::tfToMat(const tf::StampedTransform& tfT, const float scale)
{
    cv::Mat cvT = cv::Mat::eye(4, 4, CV_32F);

    cvT.at<float>(0,3) = tfT.getOrigin().x()*scale;
    cvT.at<float>(1,3) = tfT.getOrigin().y()*scale;
    cvT.at<float>(2,3) = tfT.getOrigin().z()*scale;
    cvT.at<float>(3,3) = 1.0;

    cvT.at<float>(0,0) = tfT.getBasis().getColumn(0).x();
    cvT.at<float>(1,0) = tfT.getBasis().getColumn(0).y();
    cvT.at<float>(2,0) = tfT.getBasis().getColumn(0).z();
    cvT.at<float>(0,1) = tfT.getBasis().getColumn(1).x();
    cvT.at<float>(1,1) = tfT.getBasis().getColumn(1).y();
    cvT.at<float>(2,1) = tfT.getBasis().getColumn(1).z();
    cvT.at<float>(0,2) = tfT.getBasis().getColumn(2).x();
    cvT.at<float>(1,2) = tfT.getBasis().getColumn(2).y();
    cvT.at<float>(2,2) = tfT.getBasis().getColumn(2).z();

    return cvT;
}







void SubscribeHandler::Shutdown(){

	std::cout << "ROS shutdown" << std::endl;
	ros::shutdown();
}




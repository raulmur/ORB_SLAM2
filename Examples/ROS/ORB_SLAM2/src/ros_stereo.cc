/*
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"System.h"
#include <iostream>
#include <sstream>
#include "std_msgs/Int8.h"
#include "sensor_msgs/Imu.h"

using namespace std;
cv::Mat pose;
ros::Publisher pose_pub; 
ros::Publisher track_pub; 
ros::Publisher m_pub;
ros::Publisher p_pub;
ros::Publisher test_pub2;  
ros::Publisher test_pub; 
ros::Publisher version; 

bool pubPose;
bool pubTracking;
bool pubM;
cv::Mat orientation;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

void chatterCallback(sensor_msgs::Imu matrix)
{
  //orientation = matrix;
  //tf::Quaternion q;
  //RotMatrix.getRotation(q);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD"); //This should be changed to Stereo
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    
    //advertising my publishers
    m_pub = nh.advertise<geometry_msgs::PoseStamped>("mVelocity",1000);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("camera_pose",1000);
    track_pub = nh.advertise<std_msgs::Int8>("tracking_state", 1000);
    version = nh.advertise<std_msgs::Int8>("version_1", 1000);
    test_pub = nh.advertise<std_msgs::Int8>("testing_pub", 1000);
    test_pub2 = nh.advertise<std_msgs::Int8>("testing_sub", 1000);
    p_pub = nh.advertise<geometry_msgs::PoseStamped>("pVelocity",1000);
    
    //setting up subscriber for IMU Orientation
    ros::Subscriber sub = nh.subscribe("imu/data", 1000, chatterCallback);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();
    return 0;
}

void publish(cv::Mat toPublish, ros::Publisher Publisher) {
    if (toPublish.empty()) {return;}
    
    cv::Mat Rotation = toPublish.rowRange(0,3).colRange(0,3); //getting rotation matrix
	cv::Mat Translation = toPublish.rowRange(0,3).col(3); //getting translation
	
	tf::Matrix3x3 RotationMatrix(Rotation.at<float>(0,0),Rotation.at<float>(0,1),Rotation.at<float>(0,2),
		                    Rotation.at<float>(1,0),Rotation.at<float>(1,1),Rotation.at<float>(1,2),
		                    Rotation.at<float>(2,0),Rotation.at<float>(2,1),Rotation.at<float>(2,2));

	tf::Vector3 TranslationVector(Translation.at<float>(0), Translation.at<float>(1), Translation.at<float>(2));
	
	tf::Quaternion Quaternion;           
    RotationMatrix.getRotation(Quaternion); //converting rotation matrix into quaternion
    
    tf::Transform TF_Transform = tf::Transform(RotationMatrix, TranslationVector);
    
    geometry_msgs::PoseStamped MessageToPublish;
	MessageToPublish.pose.position.x = TF_Transform.getOrigin().x();
	MessageToPublish.pose.position.y = TF_Transform.getOrigin().y();
	MessageToPublish.pose.position.z = TF_Transform.getOrigin().z();
	MessageToPublish.pose.orientation.x = TF_Transform.getRotation().x();
	MessageToPublish.pose.orientation.y = TF_Transform.getRotation().y();
	MessageToPublish.pose.orientation.z = TF_Transform.getRotation().z();
	MessageToPublish.pose.orientation.w = TF_Transform.getRotation().w();

	MessageToPublish.header.stamp = ros::Time::now();
	MessageToPublish.header.frame_id = "init_link";
	Publisher.publish(MessageToPublish); 
}    

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        pose = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
	pose =  mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    ////// publishing tracking state ///////
    /*
    int state = mpSLAM->mpTracker->mState;
    std_msgs::Int8 msg;
    msg.data = state;
    track_pub.publish(msg);
    
    ////// publishing tracking debugging ///////
    
    int testing = mpSLAM->mpTracker->my_variable; //publishing what Tracking.cc has
    std_msgs::Int8 test_int;
    test_int.data = testing;
    test_pub.publish(test_int);
    */
    
    ////// publishing tracking debugging ///////
    // demonstrates ability to change variables in tracker from stereo
    mpSLAM->mpTracker->my_variable = 5;
    int testing = mpSLAM->mpTracker->my_variable;
    std_msgs::Int8 test_int;
    test_int.data = testing;
    test_pub.publish(test_int);
    
    ////// publishing tracking debugging ///////
    
    //int testing_pub = test_int.data + 1; //publishing 1 + Tracking.cc
    int testing_pub = 5;
    std_msgs::Int8 test_int2;
    test_int2.data = testing_pub;
    test_pub2.publish(test_int2);

    //////// publishing pose and equivalent transform ///////

    // TODO: make bool to skip publishing pose instead of simply returning

    pubPose = true;
    if (pose.empty()) {pubPose = false;} //skipping if pose is empty (ex. if tracking is lost) 
    if (pubPose) {

	    cv::Mat TWC = mpSLAM->mpTracker->mCurrentFrame.mTcw.inv();  
	    cv::Mat Rotation = TWC.rowRange(0,3).colRange(0,3);  
	    cv::Mat Translation = TWC.rowRange(0,3).col(3);

	    tf::Matrix3x3 RotMatrix(Rotation.at<float>(0,0),Rotation.at<float>(0,1),Rotation.at<float>(0,2),
		                    Rotation.at<float>(1,0),Rotation.at<float>(1,1),Rotation.at<float>(1,2),
		                    Rotation.at<float>(2,0),Rotation.at<float>(2,1),Rotation.at<float>(2,2));

	    tf::Vector3 TransVect(Translation.at<float>(0), Translation.at<float>(1), Translation.at<float>(2));
	    
	    tf::Quaternion q;
	    RotMatrix.getRotation(q);

	    static tf::TransformBroadcaster br_pose;
	    tf::Transform transform = tf::Transform(RotMatrix, TransVect);
	    br_pose.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "init_link", "camera_pose"));

	    geometry_msgs::PoseStamped PubPose;
	    PubPose.pose.position.x = transform.getOrigin().x();
	    PubPose.pose.position.y = transform.getOrigin().y();
	    PubPose.pose.position.z = transform.getOrigin().z();
	    PubPose.pose.orientation.x = transform.getRotation().x();
	    PubPose.pose.orientation.y = transform.getRotation().y();
	    PubPose.pose.orientation.z = transform.getRotation().z();
	    PubPose.pose.orientation.w = transform.getRotation().w();

	    PubPose.header.stamp = ros::Time::now();
	    PubPose.header.frame_id = "init_link";
	    pose_pub.publish(PubPose);
    }
    
    //////// publishing mVelocity and equivalent transform ///////

    cv::Mat mVelocity = mpSLAM->mpTracker->mVelocity; //getting mVelocity
    pubM = true;
    if (mVelocity.empty()) {pubM = false;} //returning if mVelocity is empty (ex. if tracking is lost or just starting up) 

    if (pubM) {
	    cv::Mat mRotation = mVelocity.rowRange(0,3).colRange(0,3); //getting rotation matrix
	    cv::Mat mTranslation = mVelocity.rowRange(0,3).col(3); //getting translation

	    tf::Matrix3x3 mRotMatrix(mRotation.at<float>(0,0),mRotation.at<float>(0,1),mRotation.at<float>(0,2),
		                    mRotation.at<float>(1,0),mRotation.at<float>(1,1),mRotation.at<float>(1,2),
		                    mRotation.at<float>(2,0),mRotation.at<float>(2,1),mRotation.at<float>(2,2));

	    tf::Vector3 mTransVect(mTranslation.at<float>(0), mTranslation.at<float>(1), mTranslation.at<float>(2));

	    tf::Quaternion m;           
	    mRotMatrix.getRotation(m); //converting rotation matrix into quaternion

	    static tf::TransformBroadcaster br_m;
	    tf::Transform transform_m = tf::Transform(mRotMatrix, mTransVect);
	    br_m.sendTransform(tf::StampedTransform(transform_m, ros::Time::now(), "init_link", "mVelocity"));

	    geometry_msgs::PoseStamped PubM;
	    PubM.pose.position.x = transform_m.getOrigin().x();
	    PubM.pose.position.y = transform_m.getOrigin().y();
	    PubM.pose.position.z = transform_m.getOrigin().z();
	    PubM.pose.orientation.x = transform_m.getRotation().x();
	    PubM.pose.orientation.y = transform_m.getRotation().y();
	    PubM.pose.orientation.z = transform_m.getRotation().z();
	    PubM.pose.orientation.w = transform_m.getRotation().w();

	    PubM.header.stamp = ros::Time::now();
	    PubM.header.frame_id = "init_link";
	    m_pub.publish(PubM); 
    }
    
    
    ////// Publishing pVelocity //////
    cv::Mat pVelocity = mpSLAM->mpTracker->pVelocity; //getting mVelocity
    publish(pVelocity, p_pub);
    
} //end












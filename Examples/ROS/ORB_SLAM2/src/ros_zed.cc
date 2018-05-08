/**
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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// include for publishing ROS odometry
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void PublishOdomAndTf(const cv::Mat Tcw);


    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("orb_odom", 10);
    tf::TransformBroadcaster odom_broadcaster;
    tf::Transform transform;
    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();
    double x, y, z;
    double dx, dy, dz;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ZED");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    char WhichCamera;  
    cout << "Do you want to read stereo images or monocular image?(s/m)" << endl;  
    cin >> WhichCamera;  
    if(WhichCamera == 'S' || WhichCamera == 's'){
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

        message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/left/image_rect_color", 1);
        message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/right/image_rect_color", 1);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
        message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
        sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
        ros::spin();
        // Stop all threads
        SLAM.Shutdown();
        // Save camera trajectory
        // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
        // SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
        SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

        // Save customized Map
        char IsSaveMap;  
        cout << "Do you want to save the map?(y/n)" << endl;  
        cin >> IsSaveMap;  
        if(IsSaveMap == 'Y' || IsSaveMap == 'y')  
            SLAM.SaveMap("MapPointandKeyFrame.bin");

    }
    else if(WhichCamera == 'M' || WhichCamera == 'm'){
        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

        ImageGrabber igb(&SLAM);

        ros::NodeHandle nodeHandler;
        ros::Subscriber sub = nodeHandler.subscribe("/left/image_rect_color", 1, &ImageGrabber::GrabImage,&igb);
        ros::spin();
        // Stop all threads
        SLAM.Shutdown();
        // Save camera trajectory
        // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
        // SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
        // SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

        // Save customized Map
        char IsSaveMap;  
        cout << "Do you want to save the map?(y/n)" << endl;  
        cin >> IsSaveMap;  
        if(IsSaveMap == 'Y' || IsSaveMap == 'y')  
            SLAM.SaveMap("MapPointandKeyFrame.bin");

    }
    else{
        cerr << "Please type 'S' or 's' or 'M' or 'm'." << endl;
    }

    ros::shutdown();

    return 0;
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

    cv::Mat Tcw;

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }
    // cout << "Current timestemp = " << setprecision(18) << cv_ptrLeft->header.stamp.toSec() << ", position : " << Tcw << endl;
    if(!Tcw.empty()){
        PublishOdomAndTf(Tcw);
    }

}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw;

    Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    // cout << "Current timestemp = " << setprecision(18) << cv_ptr->header.stamp.toSec() << ", position : " << Tcw << endl;
    // cout << "Tcw.at<float>(1,2) = " << Tcw.at<float>(1,2) << endl;
    if(!Tcw.empty()){
        PublishOdomAndTf(Tcw);
    }
    
}

void ImageGrabber::PublishOdomAndTf(const cv::Mat Tcw){

    // Now start publishing odometry topic into ROS
    current_time = ros::Time::now();

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    vector<float> quat = ORB_SLAM2::Converter::toQuaternion(Rwc);

    const float MAP_SCALE = 1.0f;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(twc.at<float>(0, 0) * MAP_SCALE, twc.at<float>(0, 1) * MAP_SCALE, twc.at<float>(0, 2) * MAP_SCALE));
    tf::Quaternion quaternion(quat[0], quat[1], quat[2], quat[3]); // x,y,z,w
    transform.setRotation(quaternion);

    odom_broadcaster.sendTransform(tf::StampedTransform(transform, current_time, "orb_odom", "orb_base_link"));

    // // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    // // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    // // double delta_th = vth * dt;

    // // x += delta_x;
    // // y += delta_y;
    // // th += delta_th;


    // //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = quat[0];
    odom_quat.y = quat[1];
    odom_quat.z = quat[2];
    odom_quat.w = quat[3];

    // //first, we'll publish the transform over tf
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = current_time;
    // odom_trans.header.frame_id = "orb_odom";
    // odom_trans.child_frame_id = "orb_base_link";

    // odom_trans.transform.translation.x = -z;
    // odom_trans.transform.translation.y = -x;
    // odom_trans.transform.translation.z = -y;
    // odom_trans.transform.rotation = odom_quat;

    // //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "orb_odom";

    //set the position
    odom.pose.pose.position.x = twc.at<float>(0, 0) * MAP_SCALE;
    odom.pose.pose.position.y = twc.at<float>(0, 1) * MAP_SCALE;
    odom.pose.pose.position.z = twc.at<float>(0, 2) * MAP_SCALE;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "orb_base_link";
    // odom.twist.twist.linear.x = vx;
    // odom.twist.twist.linear.y = vy;
    // odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
}


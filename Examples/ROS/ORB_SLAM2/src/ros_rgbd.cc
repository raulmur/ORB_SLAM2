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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp> //ADR may not need this

#include"../../../include/System.h"

#include "std_msgs/Float32.h" //ADR
#include "geometry_msgs/Pose.h" //ADR
#include "geometry_msgs/TransformStamped.h"
#include "../../../include/System.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

 cv::Mat pose;
 ros::Publisher pose_pub; 

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/camera_pose",1);
	
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());


    //code that is used to publish pose

    cv::Mat TWC = mpSLAM->mpTracker->mCurrentFrame.mTcw.inv();  
    cv::Mat RWC = TWC.rowRange(0,3).colRange(0,3);  
    cv::Mat tWC = TWC.rowRange(0,3).col(3);
    
    tf::Matrix3x3 M(RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
    	            RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
    	            RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2));

    tf::Vector3 V(tWC.at<float>(0), tWC.at<float>(1), tWC.at<float>(2));

    tf::Quaternion q;
    M.getRotation(q);
    
    static tf::TransformBroadcaster br;
    tf::Transform transform = tf::Transform(M, V);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "init_link", "camera_pose"));
    
    geometry_msgs::PoseStamped _pose;
    _pose.pose.position.x = transform.getOrigin().x();
    _pose.pose.position.y = transform.getOrigin().y();
    _pose.pose.position.z = transform.getOrigin().z();
    _pose.pose.orientation.x = transform.getRotation().x();
    _pose.pose.orientation.y = transform.getRotation().y();
    _pose.pose.orientation.z = transform.getRotation().z();
    _pose.pose.orientation.w = transform.getRotation().w();
    
    _pose.header.stamp = ros::Time::now();
    _pose.header.frame_id = "init_link";
    pose_pub.publish(_pose);

}




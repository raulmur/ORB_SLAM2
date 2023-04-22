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

#include<ros/ros.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/MultiArrayDimension.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

ofstream g_ofs;
int g_seq;
ros::Publisher g_pubPose;
ros::Publisher g_pubInfo;
struct PoseInfo
{
    double numberOfMatches = 0.0;
    bool isLost = true;
};

PoseInfo g_poseInfo;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    tf::TransformBroadcaster _transformBroadcaster;
};

int main(int argc, char **argv)
{
    g_ofs.open("log.csv");
    g_seq = 0;

    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,false);

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

    g_pubPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/cube/data/vslam_localization/pose", 1);
    g_pubInfo = nh.advertise<std_msgs::Float64MultiArray>("/cube/data/vslam_localization/info", 1);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

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

    cv::Mat trackingResult;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        trackingResult = mpSLAM->TrackStereo(
            imLeft,
            imRight,
            cv_ptrLeft->header.stamp.toSec(),
            g_poseInfo.numberOfMatches,
            g_poseInfo.isLost);
    }
    else
    {
        trackingResult = mpSLAM->TrackStereo(
            cv_ptrLeft->image,
            cv_ptrRight->image,
            cv_ptrLeft->header.stamp.toSec(),
            g_poseInfo.numberOfMatches,
            g_poseInfo.isLost);

        auto vKeys = mpSLAM->GetTrackedKeyPointsUn();
        auto vMPs = mpSLAM->GetTrackedMapPoints();
        const int N = vKeys.size();
        auto leftImageToSave = cv_ptrLeft->image.clone();
        auto rightImageToSave = cv_ptrRight->image.clone();

        for(int i=0; i<N; i++)
        {
            if(vMPs[i])
            {
                cv::circle(leftImageToSave,vKeys[i].pt,1,cv::Scalar(0,255,0),-1);
                cv::circle(rightImageToSave,vKeys[i].pt,1,cv::Scalar(0,255,0),-1);
            }
        }

        static int num = 0;

        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << num;
        cv::imwrite("./data/left" + ss.str() + ".jpg", leftImageToSave);
        cv::imwrite("./data/right" + ss.str() + ".jpg", rightImageToSave);

        num++;
    }

    if (trackingResult.empty())
    {
        ROS_INFO("Tracking lost");
        return;
    }

    cv::Mat cvRotCamToInit = trackingResult.rowRange(0,3).colRange(0,3).t();
    cv::Mat cvTransCamToInit = -cvRotCamToInit * trackingResult.rowRange(0,3).col(3);

    tf::Matrix3x3 orbToRosCoord(
            0, 0, 1,
            1, 0, 0,
            0, 1, 0);
    tf::Matrix3x3 rot(
            cvRotCamToInit.at<float>(0,0), cvRotCamToInit.at<float>(0,1), cvRotCamToInit.at<float>(0,2),
            cvRotCamToInit.at<float>(1,0), cvRotCamToInit.at<float>(1,1), cvRotCamToInit.at<float>(1,2),
            cvRotCamToInit.at<float>(2,0), cvRotCamToInit.at<float>(2,1), cvRotCamToInit.at<float>(2,2));
    tf::Quaternion q;
    (orbToRosCoord * rot).getRotation(q);

    tf::Quaternion qBase;
    qBase.setRPY(0, -M_PI_2, 0);
    tf::Quaternion rosQ = q * qBase;

    tf::Vector3 trans = tf::Vector3(cvTransCamToInit.at<float>(0,0), cvTransCamToInit.at<float>(1,0), cvTransCamToInit.at<float>(2,0));
    tf::Vector3 rosTransCam = orbToRosCoord * trans;

    tf::Transform transform;
    transform.setRotation(rosQ);
    transform.setOrigin(rosTransCam);

    // Convert camera pose to vehicle pose, assuming camera and vehicle face the same direction.
    tf::Vector3 transCamToBase = tf::Vector3(-0.08, 0, 0);
    tf::Vector3 rosTransBase = transform * transCamToBase;

    geometry_msgs::PoseWithCovarianceStamped poseCovStamped;
    poseCovStamped.header.frame_id = "map";
    poseCovStamped.header.seq = g_seq;
    poseCovStamped.header.stamp = ros::Time::now();
    poseCovStamped.pose.pose.position.x = rosTransBase.x();
    poseCovStamped.pose.pose.position.y = rosTransBase.y();
    poseCovStamped.pose.pose.position.z = rosTransBase.z();
    tf::quaternionTFToMsg(rosQ, poseCovStamped.pose.pose.orientation);
    // clang-format off
    poseCovStamped.pose.covariance = {
        1e-4, 0,    0,    0,    0,    0,
        0,    1e-4, 0,    0,    0,    0,
        0,    0,    1e-4, 0,    0,    0,
        0,    0,    0,    1e-6, 0,    0,
        0,    0,    0,    0,    1e-6, 0,
        0,    0,    0,    0,    0,    1e-6
    };
    // clang-format on

    g_pubPose.publish(poseCovStamped);
    g_seq++;

    std_msgs::Float64MultiArray infoMsg;
    std::vector<double> data;
    data.push_back(poseCovStamped.header.stamp.toSec());
    data.push_back(g_poseInfo.numberOfMatches);
    g_poseInfo.isLost ? data.push_back(1.0) :
        data.push_back(0.0);
    infoMsg.data.resize(data.size());

    int count = 0;
    for_each(data.begin(), data.end(),
    [&infoMsg, &count](const double& data){
        infoMsg.data[count] = data;
        count ++;
    });

    g_pubInfo.publish(infoMsg);
//    g_ofs << trans[0] << "," << trans[1] << "," << trans[2] << "," << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << std::endl;
}



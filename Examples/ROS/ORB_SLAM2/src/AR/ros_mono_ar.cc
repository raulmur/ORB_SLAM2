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
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include"../../../include/System.h"

#include"ViewerAR.h"

using namespace std;


ORB_SLAM2::ViewerAR viewerAR;
bool bRGB = true;

cv::Mat K;
cv::Mat DistCoef;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,false);


    cout << endl << endl;
    cout << "-----------------------" << endl;
    cout << "Augmented Reality Demo" << endl;
    cout << "1) Translate the camera to initialize SLAM." << endl;
    cout << "2) Look at a planar region and translate the camera." << endl;
    cout << "3) Press Insert Cube to place a virtual cube in the plane. " << endl;
    cout << endl;
    cout << "You can place several cubes in different planes." << endl;
    cout << "-----------------------" << endl;
    cout << endl;


    viewerAR.SetSLAM(&SLAM);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);


    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
    float fps = fSettings["Camera.fps"];
    viewerAR.SetFPS(fps);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    viewerAR.SetCameraCalibration(fx,fy,cx,cy);

    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    DistCoef = cv::Mat::zeros(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

    thread tViewer = thread(&ORB_SLAM2::ViewerAR::Run,&viewerAR);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
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
    cv::Mat im = cv_ptr->image.clone();
    cv::Mat imu;
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    int state = mpSLAM->GetTrackingState();
    vector<ORB_SLAM2::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
    vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();

    cv::undistort(im,imu,K,DistCoef);

    if(bRGB)
        viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
    else
    {
        cv::cvtColor(imu,imu,CV_RGB2BGR);
        viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
    }    
}



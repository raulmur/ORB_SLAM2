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

#include <std_msgs/Float64.h>
#include <ORB_SLAM2/intVector.h>
#include <ORB_SLAM2/floatVector.h>
#include <ORB_SLAM2/float32Vector.h>
#include <ORB_SLAM2/floatMat.h>
#include <ORB_SLAM2/frame.h>
#include <ORB_SLAM2/bow.h>
#include <ORB_SLAM2/descriptor.h>
#include <ORB_SLAM2/feature.h>
#include <ORB_SLAM2/point2D.h>
#include <ORB_SLAM2/keypoint.h>


#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include"../../../include/System.h"
#include"../../../include/Frame.h"
#include"../../../Thirdparty/DBoW2/DBoW2/BowVector.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void GrabImage_a(const sensor_msgs::ImageConstPtr& msg);

    void insertKeyFrame(const ORB_SLAM2::frame &frame_msg);

    void FrameinfoToFrame(const ORB_SLAM2::frame &frame_msg, ORB_SLAM2::Frame* CurrentFramePtr);

    ORB_SLAM2::System* mpSLAM;

    ros::Publisher pub_;
    ros::Publisher pub_a_;
};


int main(int argc, char **argv)
{
    std::vector<int> v1;
    for (size_t i = 0; i < 10; i++) {
        /* code */
        v1.push_back(i);
    }
    for(std::vector<int>::const_iterator vit=v1.begin(), vend=v1.end(); vit!=vend; vit++)
    {
        std::cout << "v  = " <<*vit<< '\n';
    }
    v1.reserve(15);
    v1[13]=13;
    for(std::vector<int>::const_iterator vit=v1.begin(), vend=v1.end(); vit!=vend; vit++)
    {
        std::cout << "v  = " <<*vit<< '\n';
    }
    std::cout << "v[13]  = " <<v1[13]<< '\n';

    ros::init(argc, argv, "multiMono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub_a = nodeHandler.subscribe("/frameinfo", 1, &ImageGrabber::insertKeyFrame,&igb);
    //assistant tracking thread
    //ros::Subscriber sub_a = nodeHandler.subscribe("/camera1/image_raw", 1, &ImageGrabber::GrabImage_a,&igb);
    //publish the assistant tracking frames
    igb.pub_ = nodeHandler.advertise<ORB_SLAM2::floatMat>("framepose",10);
    igb.pub_a_ = nodeHandler.advertise<ORB_SLAM2::frame>("frameinfo",10);
    // ros::Publisher  pub = nodeHandler.advertise<ORB_SLAM2::floatVector>("/framepose",10);

    ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
    //ros::spin();

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

    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    if(Tcw.cols != 0)
    {
        ORB_SLAM2::floatMat tcw_msg;
        ORB_SLAM2::floatVector v;
        v.floatVector.push_back(Tcw.at<float>(0,0));
        v.floatVector.push_back(Tcw.at<float>(1,0));
        v.floatVector.push_back(Tcw.at<float>(2,0));

        tcw_msg.floatMat.push_back(v);
        tcw_msg.floatMat.push_back(v);
        tcw_msg.floatMat.push_back(v);
        // std::cout << "publish the tcw_msg.data = " << Tcw.at<float>(0,0) << std::endl;
        // std::cout << "Tcw = " << Tcw.cols << " x " << Tcw.rows << std::endl;
        pub_.publish(tcw_msg);
    }

}

void ImageGrabber::GrabImage_a(const sensor_msgs::ImageConstPtr& msg)
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


    ORB_SLAM2::Frame* CurrentFramePtr;
    cv::Mat Tcw = mpSLAM->TrackMonocular_a(cv_ptr->image,cv_ptr->header.stamp.toSec(),CurrentFramePtr);

    if(Tcw.cols != 0)
    {


        ORB_SLAM2::frame frame_msg;
        frame_msg.mTimeStamp = CurrentFramePtr->mTimeStamp;
        frame_msg.N = CurrentFramePtr->N;

        for(std::vector<cv::KeyPoint>::const_iterator vit=CurrentFramePtr->mvKeysUn.begin(), vend=CurrentFramePtr->mvKeysUn.end(); vit!=vend; vit++)
        {
            ORB_SLAM2::keypoint keypoint_msg;
            keypoint_msg.x = vit->pt.x;
            keypoint_msg.y = vit->pt.y;
            keypoint_msg.angle = vit->angle;
            keypoint_msg.class_id = vit->class_id;
            keypoint_msg.octave = vit->octave;
            keypoint_msg.response = vit->response;
            keypoint_msg.size = vit->size;
            frame_msg.mvKeysUn.push_back(keypoint_msg);
        }

        for(DBoW2::BowVector::const_iterator vit=CurrentFramePtr->mBowVec.begin(), vend=CurrentFramePtr->mBowVec.end(); vit!=vend; vit++)
        {
            ORB_SLAM2::bow bow_msg;
            bow_msg.WordId = vit->first;
            bow_msg.WordValue = vit->second;
            frame_msg.mBowVec.push_back(bow_msg);
        }

        for(std::map<unsigned int,std::vector<unsigned int>>::const_iterator vit=CurrentFramePtr->mFeatVec.begin(), vend=CurrentFramePtr->mFeatVec.end(); vit!=vend; vit++)
        {
            ORB_SLAM2::feature feature_msg;
            feature_msg.NodeId = vit->first;
            feature_msg.i_feature = vit->second;
            frame_msg.mFeatVec.push_back(feature_msg);
        }

        for (int i = 0; i < CurrentFramePtr->mDescriptors.rows; i++) {
            ORB_SLAM2::descriptor descriptor_msg;
            std::vector<uchar> v;
            for (int j = 0; j < CurrentFramePtr->mDescriptors.cols; j++) {
                v.push_back(CurrentFramePtr->mDescriptors.at<uchar>(i,j));
            }
            // std::cout << "mDescriptors[" << i << "]:"<< CurrentFramePtr->mDescriptors.row(i).clone()<< std::endl;
            descriptor_msg.descriptor = v;
            frame_msg.mDescriptors.push_back(descriptor_msg);
        }

        std::vector<uchar> ucharv;
        for (size_t i = 0; i < CurrentFramePtr->mvbOutlier.size(); i++) {
            if(CurrentFramePtr->mvbOutlier[i]) ucharv.push_back(1);
            else ucharv.push_back(0);
        }
        frame_msg.mvbOutlier = ucharv;

        for (int i = 0; i < CurrentFramePtr->mTcw.rows; i++) {
            ORB_SLAM2::float32Vector float32v_msg;
            for (int j = 0; j < CurrentFramePtr->mTcw.cols; j++) {
                float32v_msg.float32Vector.push_back(Tcw.at<float>(i,j));
            }
            frame_msg.mTcw.push_back(float32v_msg);
        }


        frame_msg.nNextId = CurrentFramePtr->nNextId;
        frame_msg.mnId = CurrentFramePtr->mnId;

        pub_a_.publish(frame_msg);
    }

}

void ImageGrabber::insertKeyFrame(const ORB_SLAM2::frame &frame_msg)
{
    std::cout << "get the frame_msg" << '\n';
    // mpSLAM->mpTracker_a->SetAssistant(false);
    std::cout << "now the mCurrentFrame is" << '\n';
    mpSLAM->mpTracker_a->mCurrentFrame.Show();
    ORB_SLAM2::Frame* CurrentFramePtr = new ORB_SLAM2::Frame(mpSLAM->mpTracker->mCurrentFrame);
    FrameinfoToFrame(frame_msg,CurrentFramePtr);
    mpSLAM->mpTracker_a->mCurrentFrame = *CurrentFramePtr;
    std::cout << "afrer FrameinfoToFrame now the mCurrentFrame is" << '\n';
    mpSLAM->mpTracker_a->mCurrentFrame.Show();
    std::cout << "start track sharing" << '\n';
    mpSLAM->mpTracker_a->TrackForNewFrameFromSharing();
    std::cout << "finished track sharing" << '\n';

    // mpSLAM->mpTracker_a->SetAssistant(true);
}

void ImageGrabber::FrameinfoToFrame(const ORB_SLAM2::frame &frame_msg,ORB_SLAM2::Frame* CurrentFramePtr)
{
    std::cout << "FrameinfoToFrame" << '\n';
    CurrentFramePtr->mTimeStamp = frame_msg.mTimeStamp;
    CurrentFramePtr->N = frame_msg.N;

    // CurrentFramePtr->mvpMapPoints.clear();
    CurrentFramePtr->mvpMapPoints.reserve(frame_msg.N);
    CurrentFramePtr->mvKeysUn.clear();
    CurrentFramePtr->mvKeys.clear();
    CurrentFramePtr->mvuRight.clear();
    CurrentFramePtr->mvDepth.clear();
    for(std::vector<ORB_SLAM2::keypoint>::const_iterator vit=frame_msg.mvKeysUn.begin(), vend=frame_msg.mvKeysUn.end(); vit!=vend; vit++)
    {
        cv::KeyPoint keypoint;
        keypoint.pt.x = vit->x;
        keypoint.pt.y = vit->y;
        keypoint.angle = vit->angle;
        keypoint.class_id = vit->class_id;
        keypoint.octave = vit->octave;
        keypoint.response = vit->response;
        keypoint.size = vit->size;
        CurrentFramePtr->mvKeysUn.push_back(keypoint);
        CurrentFramePtr->mvKeys.push_back(keypoint);
        CurrentFramePtr->mvuRight.push_back(-1);
        CurrentFramePtr->mvDepth.push_back(-1);
    }


    CurrentFramePtr->mBowVec.clear();
    CurrentFramePtr->mDescriptors = cv::Mat::zeros(frame_msg.N, 32, CV_8UC1);
    for (int i = 0; i < frame_msg.N; i++) {
        for (int j = 0; j < 32; j++) {
            CurrentFramePtr->mDescriptors.at<uchar>(i,j) = frame_msg.mDescriptors[i].descriptor[j];
        }
    }

    CurrentFramePtr->mvbOutlier.clear();
    CurrentFramePtr->mvbOutlier = vector<bool>(frame_msg.N,false);
    /*
    for (size_t i = 0; i < frame_msg.mvbOutlier.size(); i++) {
        if(frame_msg.mvbOutlier[i]) CurrentFramePtr->mvbOutlier.push_back(1);
        else CurrentFramePtr->mvbOutlier.push_back(0);
    }
    */

    std::cout << "start to convert cv::Mat" << '\n';
    CurrentFramePtr->SetPose(cv::Mat::eye(4,4,CV_32F));
    std::cout << "Mat size: " << CurrentFramePtr->mTcw.rows <<" x " <<CurrentFramePtr->mTcw.cols << std::endl;
    for (int i = 0; i < CurrentFramePtr->mTcw.rows; i++) {
        for (int j = 0; j < CurrentFramePtr->mTcw.cols; j++) {
            // std::cout << "mTcw.at<float>("<<i<<","<<j<<"):"<<frame_msg.mTcw[i].float32Vector[j] << '\n';
            CurrentFramePtr->mTcw.at<float>(i,j) = frame_msg.mTcw[i].float32Vector[j];
        }
    }
    std::cout << " converted cv::Mat" << '\n';



    CurrentFramePtr->mnId = CurrentFramePtr->nNextId;
    CurrentFramePtr->nNextId ++;

}

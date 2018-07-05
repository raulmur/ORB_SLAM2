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
// include for subscribing ROS IMU topic
#include "sensor_msgs/Imu.h"
#include <mutex>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

// include for IMU processing
#include <eigen3/Eigen/Dense>
#include "../../../src/IMU/imudata.h"
#include "../../../src/IMU/configparam.h"

using namespace std;
// using namespace Eigen;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void PublishOdomAndTf(const cv::Mat Tcw);
    void GrabIMU(const sensor_msgs::Imu::ConstPtr& msg);
    void GrabStereoWithIMU(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    // void ProcessIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("orb_odom", 10);
    tf::TransformBroadcaster odom_broadcaster;
    tf::Transform transform;
    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();
    ros::Time imu_last_time;
    float yaw_angle_accums = 0;
    std::mutex mMutexYawAngle;

    double x, y, z;
    double dx, dy, dz;

    // // For IMU integration:
    // Vector3d acc_0, gyr_0;
    // Vector3d Bas[(WINDOW_SIZE + 1)];
    // Vector3d Bgs[(WINDOW_SIZE + 1)];
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

    char type_in;
    bool UseIMU;
    cout << "Do you want to use the IMU subscriber for movement prediction?(y/n)" << endl;  
    cin >> type_in;
    if(type_in == 'Y' || type_in == 'y'){
        cout << "OK, let's use the IMU data. It is an experimental funtion. Good luck!" << endl;  
        UseIMU = true;
    }
    else{
        cout << "Anything not a 'Y' or 'y' means 'No'. So we won't read IMU topic." << endl; 
        UseIMU = false;
    }

    char WhichCamera;  
    cout << "Do you want to read stereo images or monocular image?(s/m)" << endl;  
    cin >> WhichCamera;  
    if(WhichCamera == 'S' || WhichCamera == 's'){
        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        bool UseViewer;
        char type_in;
        cout << "Do you want use Viewer? (y/n)" << endl;
        cin >> type_in;
        if(type_in == 'Y' || type_in == 'y'){  
            UseViewer = true;
        }
        else{
            UseViewer = false;
        }
        ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO, UseViewer);

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
        ros::Subscriber sub_imu;

        message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/left/image_rect_color", 1);
        message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/right/image_rect_color", 1);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
        message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
        // sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
        

        if(UseIMU){
            sub_imu = nh.subscribe("/imu/imu", 1, &ImageGrabber::GrabIMU, &igb);
            sync.registerCallback(boost::bind(&ImageGrabber::GrabStereoWithIMU,&igb,_1,_2));
            SLAM.SetViewerIMUFlagTrue();
        }
        else{
            sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
            SLAM.SetViewerIMUFlagFalse();
        }
        // ros::Subscriber sub_imu = nh.subscribe("/imu/imu", 1, &ImageGrabber::GrabIMU, &igb);
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
        bool UseViewer;
        char type_in;
        cout << "Do you want to use Viewer? (y/n)" << endl;
        cin >> type_in;
        if(type_in == 'Y' || type_in == 'y'){  
            UseViewer = true;
        }
        else{
            UseViewer = false;
        }
        ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, UseViewer);

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
        mpSLAM->SetSaveImageFlag();// This is just telling the viewer that it can save an image. Not actually saving one. The finall decision would be made my the viewer.
    }
    else
    {
        Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
        mpSLAM->SetSaveImageFlag();// This is just telling the viewer that it can save an image. Not actually saving one. The finall decision would be made my the viewer.
    }
    // cout << "Current timestemp = " << setprecision(18) << cv_ptrLeft->header.stamp.toSec() << ", position : " << Tcw << endl;
    if(!Tcw.empty()){
        PublishOdomAndTf(Tcw);
    }

}

void ImageGrabber::GrabStereoWithIMU(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
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

    //get current yaw_angle_accums
    float current_yaw_angle_accums;
    {
        unique_lock<mutex> lock(mMutexYawAngle);
        current_yaw_angle_accums = yaw_angle_accums;
        yaw_angle_accums = 0;
        // ROS_INFO("Imu yaw_angle Updated!!!!!!! \n");
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw = mpSLAM->TrackStereoWithIMU(imLeft, imRight, cv_ptrLeft->header.stamp.toSec(), current_yaw_angle_accums);
        mpSLAM->SetSaveImageFlag(); // This is just telling the viewer that it can save an image. Not actually saving one. The finall decision would be made my the viewer.
    }
    else
    {
        Tcw = mpSLAM->TrackStereoWithIMU(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec(), current_yaw_angle_accums);
        mpSLAM->SetSaveImageFlag();// This is just telling the viewer that it can save an image. Not actually saving one. The finall decision would be made my the viewer.
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

void ImageGrabber::GrabIMU(const sensor_msgs::Imu::ConstPtr& msg)
{
    unique_lock<mutex> lock(mMutexYawAngle);
    if(!imu_last_time.toSec()){ // Initialize imu_last_time
        imu_last_time = msg->header.stamp;
    }
    // ROS_INFO("Imu heard: seq = [%i], time = [%i, %i], frame_id = [%s]", msg->header.seq, msg->header.stamp.sec, msg->header.stamp.nsec, msg->header.frame_id.c_str());
    // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    // ROS_INFO("Imu angular_velocity x: [%f], y: [%f], z: [%f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    

    ros::Duration imu_duration = msg->header.stamp - imu_last_time;
    // TO DO: subtract bias : - ba[xxx];  - bg[xxx];
    
    float dx = msg->linear_acceleration.x; // X acceleration
    float dy = msg->linear_acceleration.y; // Y acceleration
    float dz = msg->linear_acceleration.z; // Z acceleration

    float rx = msg->angular_velocity.x; // roll_rate  
    float ry = msg->angular_velocity.y; // pitch_rate
    float rz = msg->angular_velocity.z; // yaw_rate
    
    float dt = imu_duration.toSec();
    float yaw_angle = dt*rz;
    yaw_angle_accums = yaw_angle_accums + yaw_angle;
    // ROS_INFO("Imu timestamp duration is : [%6.6f]", imu_duration.toSec());
    // ROS_INFO("In my calculation, rz = : [%f], dt = : [%f], yaw_angle = : [%f].", rz, dt, yaw_angle);
    // ROS_INFO("yaw_angle_accums = : [%f].", yaw_angle_accums);

    imu_last_time = msg->header.stamp;

    // ProcessIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}


// void ImageGrabber::ProcessIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity){
//     if (!first_imu) { 
//         first_imu = true; 
//         acc_0 = linear_acceleration; 
//         gyr_0 = angular_velocity; 
//     } 

//     if (!pre_integrations[frame_count]) { 
//         pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]}; 
//     } 

//     if (frame_count != 0) { 
//         pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity); 
//         if(solver_flag != NON_LINEAR) //comments because of recovering 
//         tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity); 

//         dt_buf[frame_count].push_back(dt); linear_acceleration_buf[frame_count].push_back(linear_acceleration); 
//         angular_velocity_buf[frame_count].push_back(angular_velocity); 

//         //midpoint integration 
//         { 
//             Vector3d g{0,0,GRAVITY}; 
//             int j = frame_count; 
//             Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g; 
//             Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j]; 
//             Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix(); 
//             Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g; 
//             Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1); 
//             Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc; 
//             Vs[j] += dt * un_acc; 
//         } 
//     } 
//     acc_0 = linear_acceleration; 
//     gyr_0 = angular_velocity;
// }

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


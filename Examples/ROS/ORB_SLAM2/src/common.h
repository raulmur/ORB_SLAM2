#ifndef COMMON_H
#define COMMON_H

#include<ros/ros.h>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include<opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>

using namespace std;

static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0.1, 0, 0,
    0, 0, 0, 0, 0.1, 0,
    0, 0, 0, 0, 0, 0.1 } };

namespace common{
inline void CreateOdomMsg(nav_msgs::Odometry& odom,
                          const sensor_msgs::ImageConstPtr& msgRGB,
                          cv::Mat cvTCW)
{
  /* Get transform from Cameta to World and translate it into a nav_msgs.Odom */
  // Invert to get the TWC
  cv::Mat cvTWC = cvTCW.inv();
  // Open CV mat to Eigen matrix (float)
  Eigen::Matrix4f T_eigen_f;
  cv::cv2eigen(cvTWC, T_eigen_f);
  // Eigen matrix (float) to Eigen matrix (double)
  Eigen::Matrix4d T_eigen_d = T_eigen_f.cast<double>();
  // Extracting and orthonormalizing the rotation matrix
  Eigen::Matrix3d R_unnormalized = T_eigen_d.block<3, 3>(0, 0);
  Eigen::AngleAxisd aa(R_unnormalized);
  Eigen::Matrix3d R = aa.toRotationMatrix();
  Eigen::Quaterniond eigQuat(R);
  // Extract the translation
  Eigen::Vector3d T(T_eigen_d.block<3, 1>(0, 3));
  
  // Format the Odom msg
  odom.header.stamp = msgRGB->header.stamp;
  odom.header.frame_id = msgRGB->header.frame_id;

  odom.pose.pose.position.x = T(0);
  odom.pose.pose.position.y = T(1);
  odom.pose.pose.position.z = T(2);

  odom.pose.pose.orientation.x = eigQuat.x();
  odom.pose.pose.orientation.y = eigQuat.y();
  odom.pose.pose.orientation.z = eigQuat.z();
  odom.pose.pose.orientation.w = eigQuat.w();
}


//
//   for( int i = 0; i < 6; i++ )
//   {
//     for( int j = 0; j < 6; j++ ) {
//       odom.pose.covariance[ i*6 + j ] = poseCovariance(i,j);
//     }
//   }
// }
}
#endif // COMMON_H

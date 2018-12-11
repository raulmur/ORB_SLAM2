#include<ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0.1, 0, 0,
    0, 0, 0, 0, 0.1, 0,
    0, 0, 0, 0, 0, 0.1 } };

inline void CreateOdomMsg(nav_msgs::Odometry& odom,
                   const sensor_msgs::ImageConstPtr& msgRGB,
                   Eigen::Vector3d T, Eigen::Quaterniond eigQuat,
                   Eigen::MatrixXd poseCovariance)
{
  /* Get transform from Cameta to World and translate it into a nav_msgs.Odom */
  odom.header.stamp = msgRGB->header.stamp;
  odom.header.frame_id = msgRGB->header.frame_id;

  odom.pose.pose.position.x = T(0);
  odom.pose.pose.position.y = T(1);
  odom.pose.pose.position.z = T(2);

  // TODO check Eigen to TF Quaternion
  odom.pose.pose.orientation.x = eigQuat.x();
  odom.pose.pose.orientation.y = eigQuat.y();
  odom.pose.pose.orientation.z = eigQuat.z();
  odom.pose.pose.orientation.w = eigQuat.w();

  for( int i = 0; i < 6; i++ )
  {
    for( int j = 0; j < 6; j++ ) {
      odom.pose.covariance[ i*6 + j ] = poseCovariance(i,j);
    }
  }
}

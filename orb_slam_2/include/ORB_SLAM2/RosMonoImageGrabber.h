#ifndef ORB_SLAM2_ROSMONOIMAGEGRABBER_H_
#define ORB_SLAM2_ROSMONOIMAGEGRABBER_H_

#include <cv_bridge/cv_bridge.h>

namespace ORB_SLAM2 {

class System;

class RosMonoImageGrabber
{
 public:
  RosMonoImageGrabber(ORB_SLAM2::System* pSLAM);

  void GrabImage(const sensor_msgs::ImageConstPtr& msg);

 private:
  ORB_SLAM2::System* mpSLAM;
};

}  // namespace ORB_SLAM2

#endif  // ORB_SLAM2_ROSMONOIMAGEGRABBER_H_

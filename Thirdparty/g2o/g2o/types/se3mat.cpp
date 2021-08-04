#include "se3mat.h"

namespace g2o {


void SE3mat::Retract(const Eigen::Vector3d dr, const Eigen::Vector3d &dt)
{
    t += R*dt;
    R = R*ExpSO3(dr);
}

Eigen::Matrix3d SE3mat::ExpSO3(const Eigen::Vector3d r)
{
    Eigen::Matrix3d W;
    W << 0, -r[2], r[1],
         r[2], 0, -r[0],
         -r[1], r[0], 0;

    const double theta = r.norm();

    if(theta<1e-6)
        return Eigen::Matrix3d::Identity() + W + 0.5l*W*W;
    else
        return Eigen::Matrix3d::Identity() + W*sin(theta)/theta + W*W*(1-cos(theta))/(theta*theta);
}

Eigen::Vector3d SE3mat::LogSO3(const Eigen::Matrix3d R)
{
    const double tr = R(0,0)+R(1,1)+R(2,2);
    const double theta = acos((tr-1.0l)*0.5l);
    Eigen::Vector3d w;
    w << R(2,1), R(0,2), R(1,0);
    if(theta<1e-6)
        return w;
    else
        return theta*w/sin(theta);
}

} //namespace g2o

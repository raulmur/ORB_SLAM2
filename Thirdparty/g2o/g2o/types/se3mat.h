#ifndef SE3mat_H
#define SE3mat_H

#include<eigen3/Eigen/Geometry>
#include<eigen3/Eigen/Core>

namespace g2o {

class SE3mat
{
public:
    SE3mat(){
        R = Eigen::Matrix3d::Identity();
        t.setZero();
    }

    SE3mat(const Eigen::Matrix3d &R_, const Eigen::Vector3d &t_):R(R_),t(t_){}

    void Retract(const Eigen::Vector3d dr, const Eigen::Vector3d &dt);

    inline Eigen::Vector3d operator* (const Eigen::Vector3d& v) const {
      return R*v + t;
    }

    inline SE3mat& operator*= (const SE3mat& T2){
      t+=R*T2.t;
      R*=T2.R;
      return *this;
    }

    inline SE3mat inverse() const{
      Eigen::Matrix3d Rt = R.transpose();
      return SE3mat(Rt,-Rt*t);
    }

protected:
    Eigen::Vector3d t;
    Eigen::Matrix3d R;

public:
    static Eigen::Matrix3d ExpSO3(const Eigen::Vector3d r);
    static Eigen::Vector3d LogSO3(const Eigen::Matrix3d R);
};

}//namespace g2o

#endif // SE3mat_H

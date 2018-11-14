#ifndef TVISLAM_IMUPREINTEGRATOR_H
#define TVISLAM_IMUPREINTEGRATOR_H

#include <Eigen/Dense>

#include "IMU/imudata.h"
#include "so3.h"

namespace ORB_SLAM2
{

using namespace Eigen;
using namespace Sophus;

typedef Eigen::Matrix<double, 9, 9> Matrix9d;

class IMUPreintegrator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUPreintegrator();
    IMUPreintegrator(const IMUPreintegrator& pre);

    //IMUPreintegrator& operator = (const IMUPreintegrator& pre);

    // reset to initial state
    void reset();

    // incrementally update 1)delta measurements, 2)jacobians, 3)covariance matrix
    void update(const Vector3d& omega, const Vector3d& acc, const double& dt);

    // delta measurements, position/velocity/rotation(matrix)
    inline Eigen::Vector3d getDeltaP() const    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    {
        return _delta_P;
    }
        inline Eigen::Vector3d getDeltaV() const    // V_k+1 = V_k + R_k*a_k*dt
    {
        return _delta_V;
    }
        inline Eigen::Matrix3d getDeltaR() const   // R_k+1 = R_k*exp(w_k*dt).     NOTE: Rwc, Rwc'=Rwc*[w_body]x
    {
        return _delta_R;
    }

    // jacobian of delta measurements w.r.t bias of gyro/acc
    inline Eigen::Matrix3d getJPBiasg() const     // position / gyro
    {
        return _J_P_Biasg;
    }
    inline Eigen::Matrix3d getJPBiasa() const     // position / acc
    {
        return _J_P_Biasa;
    }
    inline Eigen::Matrix3d getJVBiasg() const     // velocity / gyro
    {
        return _J_V_Biasg;
    }
    inline Eigen::Matrix3d getJVBiasa() const     // velocity / acc
    {
        return _J_V_Biasa;
    }
    inline Eigen::Matrix3d getJRBiasg() const  // rotation / gyro
    {
        return _J_R_Biasg;
    }

    // noise covariance propagation of delta measurements
    // note: the order is rotation-velocity-position here
    inline Matrix9d getCovPVPhi() const 
    {
        return _cov_P_V_Phi;
    }

    inline double getDeltaTime() const {
        return _delta_time;
    }

    // skew-symmetric matrix
    static Matrix3d skew(const Vector3d& v)
    {
        return SO3::hat( v );
    }
    
    // exponential map from vec3 to mat3x3 (Rodrigues formula)
    static Matrix3d Expmap(const Vector3d& v)
    {
        return SO3::exp(v).matrix();
    }
    
    // right jacobian of SO(3)
    static Matrix3d JacobianR(const Vector3d& w)
    {
        Matrix3d Jr = Matrix3d::Identity();
        double theta = w.norm();
        if(theta<0.00001)
        {
            return Jr;// = Matrix3d::Identity();
        }
        else
        {
            Vector3d k = w.normalized();  // k - unit direction vector of w
            Matrix3d K = skew(k);
            Jr =   Matrix3d::Identity()
                    - (1-cos(theta))/theta*K
                    + (1-sin(theta)/theta)*K*K;
        }
        return Jr;
    }
    static Matrix3d JacobianRInv(const Vector3d& w)
    {
        Matrix3d Jrinv = Matrix3d::Identity();
        double theta = w.norm();

        // very small angle
        if(theta < 0.00001)
        {
            return Jrinv;
        }
        else
        {
            Vector3d k = w.normalized();  // k - unit direction vector of w
            Matrix3d K = SO3::hat(k);
            Jrinv = Matrix3d::Identity()
                    + 0.5*SO3::hat(w)
                    + ( 1.0 - (1.0+cos(theta))*theta / (2.0*sin(theta)) ) *K*K;
        }

        return Jrinv;
        /*
     * in gtsam:
     *
     *   double theta2 = omega.dot(omega);
     *  if (theta2 <= std::numeric_limits<double>::epsilon()) return I_3x3;
     *  double theta = std::sqrt(theta2);  // rotation angle
     *  * Right Jacobian for Log map in SO(3) - equation (10.86) and following equations in
     *   * G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
     *   * logmap( Rhat * expmap(omega) ) \approx logmap( Rhat ) + Jrinv * omega
     *   * where Jrinv = LogmapDerivative(omega);
     *   * This maps a perturbation on the manifold (expmap(omega))
     *   * to a perturbation in the tangent space (Jrinv * omega)
     *
     *  const Matrix3 W = skewSymmetric(omega); // element of Lie algebra so(3): W = omega^
     *  return I_3x3 + 0.5 * W +
     *         (1 / (theta * theta) - (1 + cos(theta)) / (2 * theta * sin(theta))) *
     *             W * W;
     *
     * */
    }

    // left jacobian of SO(3), Jl(x) = Jr(-x)
    static Matrix3d JacobianL(const Vector3d& w)
    {
        return JacobianR(-w);
    }
    // left jacobian inverse
    static Matrix3d JacobianLInv(const Vector3d& w)
    {
        return JacobianRInv(-w);
    }


    inline Quaterniond normalizeRotationQ(const Quaterniond& r)
    {
        Quaterniond _r(r);
        if (_r.w()<0)
        {
            _r.coeffs() *= -1;
        }
        return _r.normalized();
    }

    inline Matrix3d normalizeRotationM (const Matrix3d& R)
    {
        Quaterniond qr(R);
        return normalizeRotationQ(qr).toRotationMatrix();
    }
private:
    /*
     * NOTE:
     * don't add pointer as member variable.
     * operator = is used in g2o
    */

    // delta measurements, position/velocity/rotation(matrix)
    Eigen::Vector3d _delta_P;    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    Eigen::Vector3d _delta_V;    // V_k+1 = V_k + R_k*a_k*dt
    Eigen::Matrix3d _delta_R;    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

    // jacobian of delta measurements w.r.t bias of gyro/acc
    Eigen::Matrix3d _J_P_Biasg;     // position / gyro
    Eigen::Matrix3d _J_P_Biasa;     // position / acc
    Eigen::Matrix3d _J_V_Biasg;     // velocity / gyro
    Eigen::Matrix3d _J_V_Biasa;     // velocity / acc
    Eigen::Matrix3d _J_R_Biasg;   // rotation / gyro

    // noise covariance propagation of delta measurements
    Matrix9d _cov_P_V_Phi;

    double _delta_time;

};

}

#endif // TVISLAM_IMUPREINTEGRATOR_H

/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "KannalaBrandt8.h"

#include <boost/serialization/export.hpp>

namespace ORB_SLAM3 {

    cv::Point2f KannalaBrandt8::project(const cv::Point3f &p3D) {
        const float x2_plus_y2 = p3D.x * p3D.x + p3D.y * p3D.y;
        const float theta = atan2f(sqrtf(x2_plus_y2), p3D.z);
        const float psi = atan2f(p3D.y, p3D.x);

        const float theta2 = theta * theta;
        const float theta3 = theta * theta2;
        const float theta5 = theta3 * theta2;
        const float theta7 = theta5 * theta2;
        const float theta9 = theta7 * theta2;
        const float r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5
                        + mvParameters[6] * theta7 + mvParameters[7] * theta9;

        return cv::Point2f(mvParameters[0] * r * cos(psi) + mvParameters[2],
                           mvParameters[1] * r * sin(psi) + mvParameters[3]);

    }

    cv::Point2f KannalaBrandt8::project(const cv::Matx31f &m3D) {
        return this->project(cv::Point3f(m3D(0),m3D(1),m3D(2)));
    }

    cv::Point2f KannalaBrandt8::project(const cv::Mat &m3D) {
        const float* p3D = m3D.ptr<float>();

        return this->project(cv::Point3f(p3D[0],p3D[1],p3D[2]));
    }

    Eigen::Vector2d KannalaBrandt8::project(const Eigen::Vector3d &v3D) {
        const double x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
        const double theta = atan2f(sqrtf(x2_plus_y2), v3D[2]);
        const double psi = atan2f(v3D[1], v3D[0]);

        const double theta2 = theta * theta;
        const double theta3 = theta * theta2;
        const double theta5 = theta3 * theta2;
        const double theta7 = theta5 * theta2;
        const double theta9 = theta7 * theta2;
        const double r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5
                        + mvParameters[6] * theta7 + mvParameters[7] * theta9;

        Eigen::Vector2d res;
        res[0] = mvParameters[0] * r * cos(psi) + mvParameters[2];
        res[1] = mvParameters[1] * r * sin(psi) + mvParameters[3];

        return res;
    }

    cv::Mat KannalaBrandt8::projectMat(const cv::Point3f &p3D) {
        cv::Point2f point = this->project(p3D);
        cv::Mat ret = (cv::Mat_<float>(2,1) << point.x, point.y);
        return ret.clone();
    }

    float KannalaBrandt8::uncertainty2(const Eigen::Matrix<double,2,1> &p2D)
    {
        return 1.f;
    }

    cv::Mat KannalaBrandt8::unprojectMat(const cv::Point2f &p2D){
        cv::Point3f ray = this->unproject(p2D);
        cv::Mat ret = (cv::Mat_<float>(3,1) << ray.x, ray.y, ray.z);
        return ret.clone();
    }

        cv::Matx31f KannalaBrandt8::unprojectMat_(const cv::Point2f &p2D) {
            cv::Point3f ray = this->unproject(p2D);
            cv::Matx31f r{ray.x, ray.y, ray.z};
            return r;
    }

    cv::Point3f KannalaBrandt8::unproject(const cv::Point2f &p2D) {
        //Use Newthon method to solve for theta with good precision (err ~ e-6)
        cv::Point2f pw((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1]);
        float scale = 1.f;
        float theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);
        theta_d = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f);

        if (theta_d > 1e-8) {
            //Compensate distortion iteratively
            float theta = theta_d;

            for (int j = 0; j < 10; j++) {
                float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 =
                        theta4 * theta4;
                float k0_theta2 = mvParameters[4] * theta2, k1_theta4 = mvParameters[5] * theta4;
                float k2_theta6 = mvParameters[6] * theta6, k3_theta8 = mvParameters[7] * theta8;
                float theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                                  (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
                theta = theta - theta_fix;
                if (fabsf(theta_fix) < precision)
                    break;
            }
            //scale = theta - theta_d;
            scale = std::tan(theta) / theta_d;
        }

        return cv::Point3f(pw.x * scale, pw.y * scale, 1.f);
    }

    cv::Mat KannalaBrandt8::projectJac(const cv::Point3f &p3D) {
        float x2 = p3D.x * p3D.x, y2 = p3D.y * p3D.y, z2 = p3D.z * p3D.z;
        float r2 = x2 + y2;
        float r = sqrt(r2);
        float r3 = r2 * r;
        float theta = atan2(r, p3D.z);

        float theta2 = theta * theta, theta3 = theta2 * theta;
        float theta4 = theta2 * theta2, theta5 = theta4 * theta;
        float theta6 = theta2 * theta4, theta7 = theta6 * theta;
        float theta8 = theta4 * theta4, theta9 = theta8 * theta;

        float f = theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] + theta7 * mvParameters[6] +
                  theta9 * mvParameters[7];
        float fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 + 7 * mvParameters[6] * theta6 +
                   9 * mvParameters[7] * theta8;

        cv::Mat Jac(2, 3, CV_32F);
        Jac.at<float>(0, 0) = mvParameters[0] * (fd * p3D.z * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
        Jac.at<float>(1, 0) =
                mvParameters[1] * (fd * p3D.z * p3D.y * p3D.x / (r2 * (r2 + z2)) - f * p3D.y * p3D.x / r3);

        Jac.at<float>(0, 1) =
                mvParameters[0] * (fd * p3D.z * p3D.y * p3D.x / (r2 * (r2 + z2)) - f * p3D.y * p3D.x / r3);
        Jac.at<float>(1, 1) = mvParameters[1] * (fd * p3D.z * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

        Jac.at<float>(0, 2) = -mvParameters[0] * fd * p3D.x / (r2 + z2);
        Jac.at<float>(1, 2) = -mvParameters[1] * fd * p3D.y / (r2 + z2);

        std::cout << "CV JAC: " << Jac << std::endl;

        return Jac.clone();
    }

    Eigen::Matrix<double, 2, 3> KannalaBrandt8::projectJac(const Eigen::Vector3d &v3D) {
        double x2 = v3D[0] * v3D[0], y2 = v3D[1] * v3D[1], z2 = v3D[2] * v3D[2];
        double r2 = x2 + y2;
        double r = sqrt(r2);
        double r3 = r2 * r;
        double theta = atan2(r, v3D[2]);

        double theta2 = theta * theta, theta3 = theta2 * theta;
        double theta4 = theta2 * theta2, theta5 = theta4 * theta;
        double theta6 = theta2 * theta4, theta7 = theta6 * theta;
        double theta8 = theta4 * theta4, theta9 = theta8 * theta;

        double f = theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] + theta7 * mvParameters[6] +
                  theta9 * mvParameters[7];
        double fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 + 7 * mvParameters[6] * theta6 +
                   9 * mvParameters[7] * theta8;

        Eigen::Matrix<double, 2, 3> JacGood;
        JacGood(0, 0) = mvParameters[0] * (fd * v3D[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
        JacGood(1, 0) =
                mvParameters[1] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);

        JacGood(0, 1) =
                mvParameters[0] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);
        JacGood(1, 1) = mvParameters[1] * (fd * v3D[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

        JacGood(0, 2) = -mvParameters[0] * fd * v3D[0] / (r2 + z2);
        JacGood(1, 2) = -mvParameters[1] * fd * v3D[1] / (r2 + z2);

        return JacGood;
    }

    cv::Mat KannalaBrandt8::unprojectJac(const cv::Point2f &p2D) {
        return cv::Mat();
    }

    bool KannalaBrandt8::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                                          cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated){
        if(!tvr){
            cv::Mat K = this->toK();
            tvr = new TwoViewReconstruction(K);
        }

        //Correct FishEye distortion
        std::vector<cv::KeyPoint> vKeysUn1 = vKeys1, vKeysUn2 = vKeys2;
        std::vector<cv::Point2f> vPts1(vKeys1.size()), vPts2(vKeys2.size());

        for(size_t i = 0; i < vKeys1.size(); i++) vPts1[i] = vKeys1[i].pt;
        for(size_t i = 0; i < vKeys2.size(); i++) vPts2[i] = vKeys2[i].pt;

        cv::Mat D = (cv::Mat_<float>(4,1) << mvParameters[4], mvParameters[5], mvParameters[6], mvParameters[7]);
        cv::Mat R = cv::Mat::eye(3,3,CV_32F);
        cv::Mat K = this->toK();
        cv::fisheye::undistortPoints(vPts1,vPts1,K,D,R,K);
        cv::fisheye::undistortPoints(vPts2,vPts2,K,D,R,K);

        for(size_t i = 0; i < vKeys1.size(); i++) vKeysUn1[i].pt = vPts1[i];
        for(size_t i = 0; i < vKeys2.size(); i++) vKeysUn2[i].pt = vPts2[i];

        return tvr->Reconstruct(vKeysUn1,vKeysUn2,vMatches12,R21,t21,vP3D,vbTriangulated);
    }


    cv::Mat KannalaBrandt8::toK() {
        cv::Mat K = (cv::Mat_<float>(3, 3)
                << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
        return K;
    }

    cv::Matx33f KannalaBrandt8::toK_() {
        cv::Matx33f K{mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f};

        return K;
    }

    bool KannalaBrandt8::epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &R12, const cv::Mat &t12, const float sigmaLevel, const float unc) {
        cv::Mat p3D;
        return this->TriangulateMatches(pCamera2,kp1,kp2,R12,t12,sigmaLevel,unc,p3D) > 0.0001f;
    }

    bool KannalaBrandt8::epipolarConstrain_(GeometricCamera* pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Matx33f &R12, const cv::Matx31f &t12, const float sigmaLevel, const float unc) {
        cv::Matx31f p3D;
        return this->TriangulateMatches_(pCamera2,kp1,kp2,R12,t12,sigmaLevel,unc,p3D) > 0.0001f;
    }

    bool KannalaBrandt8::matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, GeometricCamera* pOther,
                                             cv::Mat& Tcw1, cv::Mat& Tcw2,
                                            const float sigmaLevel1, const float sigmaLevel2,
                                            cv::Mat& x3Dtriangulated){
        cv::Mat Rcw1 = Tcw1.colRange(0,3).rowRange(0,3);
        cv::Mat Rwc1 = Rcw1.t();
        cv::Mat tcw1 = Tcw1.rowRange(0,3).col(3);

        cv::Mat Rcw2 = Tcw2.colRange(0,3).rowRange(0,3);
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = Tcw2.rowRange(0,3).col(3);

        cv::Point3f ray1c = this->unproject(kp1.pt);
        cv::Point3f ray2c = pOther->unproject(kp2.pt);

        cv::Mat r1(3,1,CV_32F);
        r1.at<float>(0) = ray1c.x;
        r1.at<float>(1) = ray1c.y;
        r1.at<float>(2) = ray1c.z;

        cv::Mat r2(3,1,CV_32F);
        r2.at<float>(0) = ray2c.x;
        r2.at<float>(1) = ray2c.y;
        r2.at<float>(2) = ray2c.z;

        //Check parallax between rays
        cv::Mat ray1 = Rwc1*r1;
        cv::Mat ray2 = Rwc2*r2;

        const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

        //If parallax is lower than 0.9998, reject this match
        if(cosParallaxRays > 0.9998){
            return false;
        }

        //Parallax is good, so we try to triangulate
        cv::Point2f p11,p22;

        p11.x = ray1c.x;
        p11.y = ray1c.y;

        p22.x = ray2c.x;
        p22.y = ray2c.y;

        cv::Mat x3D;

        Triangulate(p11,p22,Tcw1,Tcw2,x3D);

        cv::Mat x3Dt = x3D.t();

        //Check triangulation in front of cameras
        float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
        if(z1<=0){  //Point is not in front of the first camera
            return false;
        }


        float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
        if(z2<=0){ //Point is not in front of the first camera
            return false;
        }

        //Check reprojection error in first keyframe
        //  -Transform point into camera reference system
        cv::Mat x3D1 = Rcw1 * x3D + tcw1;
        cv::Point2f uv1 = this->project(x3D1);

        float errX1 = uv1.x - kp1.pt.x;
        float errY1 = uv1.y - kp1.pt.y;

        if((errX1*errX1+errY1*errY1)>5.991*sigmaLevel1){   //Reprojection error is high
            return false;
        }

        //Check reprojection error in second keyframe;
        //  -Transform point into camera reference system
        cv::Mat x3D2 = Rcw2 * x3D + tcw2;
        cv::Point2f uv2 = pOther->project(x3D2);

        float errX2 = uv2.x - kp2.pt.x;
        float errY2 = uv2.y - kp2.pt.y;

        if((errX2*errX2+errY2*errY2)>5.991*sigmaLevel2){   //Reprojection error is high
            return false;
        }

        //Since parallax is big enough and reprojection errors are low, this pair of points
        //can be considered as a match
        x3Dtriangulated = x3D.clone();

        return true;
    }

    float KannalaBrandt8::TriangulateMatches(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &R12, const cv::Mat &t12, const float sigmaLevel, const float unc, cv::Mat& p3D) {
        cv::Mat r1 = this->unprojectMat(kp1.pt);
        cv::Mat r2 = pCamera2->unprojectMat(kp2.pt);

        //Check parallax
        cv::Mat r21 = R12*r2;

        const float cosParallaxRays = r1.dot(r21)/(cv::norm(r1)*cv::norm(r21));

        if(cosParallaxRays > 0.9998){
            return -1;
        }

        //Parallax is good, so we try to triangulate
        cv::Point2f p11,p22;
        const float* pr1 = r1.ptr<float>();
        const float* pr2 = r2.ptr<float>();

        p11.x = pr1[0];
        p11.y = pr1[1];

        p22.x = pr2[0];
        p22.y = pr2[1];

        cv::Mat x3D;
        cv::Mat Tcw1 = (cv::Mat_<float>(3,4) << 1.f,0.f,0.f,0.f,
                                                0.f,1.f,0.f,0.f,
                                                0.f,0.f,1.f,0.f);
        cv::Mat Tcw2;
        cv::Mat R21 = R12.t();
        cv::Mat t21 = -R21*t12;
        cv::hconcat(R21,t21,Tcw2);

        Triangulate(p11,p22,Tcw1,Tcw2,x3D);
        cv::Mat x3Dt = x3D.t();

        float z1 = x3D.at<float>(2);
        if(z1 <= 0){
            return -1;
        }

        float z2 = R21.row(2).dot(x3Dt)+t21.at<float>(2);
        if(z2<=0){
            return -1;
        }

        //Check reprojection error
        cv::Point2f uv1 = this->project(x3D);

        float errX1 = uv1.x - kp1.pt.x;
        float errY1 = uv1.y - kp1.pt.y;

        if((errX1*errX1+errY1*errY1)>5.991 * sigmaLevel){   //Reprojection error is high
            return -1;
        }

        cv::Mat x3D2 = R21 * x3D + t21;
        cv::Point2f uv2 = pCamera2->project(x3D2);

        float errX2 = uv2.x - kp2.pt.x;
        float errY2 = uv2.y - kp2.pt.y;

        if((errX2*errX2+errY2*errY2)>5.991 * unc){   //Reprojection error is high
            return -1;
        }

        p3D = x3D.clone();

        return z1;
    }

    float KannalaBrandt8::TriangulateMatches_(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Matx33f &R12, const cv::Matx31f &t12, const float sigmaLevel, const float unc, cv::Matx31f& p3D) {
        cv::Matx31f r1 = this->unprojectMat_(kp1.pt);
        cv::Matx31f r2 = pCamera2->unprojectMat_(kp2.pt);

        //Check parallax
        cv::Matx31f r21 = R12*r2;

        const float cosParallaxRays = r1.dot(r21)/(cv::norm(r1)*cv::norm(r21));

        if(cosParallaxRays > 0.9998){
            return -1;
        }

        //Parallax is good, so we try to triangulate
        cv::Point2f p11,p22;

        p11.x = r1(0);
        p11.y = r1(1);

        p22.x = r2(0);
        p22.y = r2(1);

        cv::Matx31f x3D;
        cv::Matx44f Tcw1{1.f,0.f,0.f,0.f,
                0.f,1.f,0.f,0.f,
                0.f,0.f,1.f,0.f};

        cv::Matx33f R21 = R12.t();
        cv::Matx31f t21 = -R21*t12;

        cv::Matx44f Tcw2{R21(0,0),R21(0,1),R21(0,2),t21(0),
                         R21(1,0),R21(1,1),R21(1,2),t21(1),
                         R21(2,0),R21(2,1),R21(2,2),t21(2),
                         0.f,0.f,0.f,1.f};

        Triangulate_(p11,p22,Tcw1,Tcw2,x3D);
        cv::Matx13f x3Dt = x3D.t();

        float z1 = x3D(2);
        if(z1 <= 0){
            return -1;
        }

        float z2 = R21.row(2).dot(x3Dt)+t21(2);
        if(z2<=0){
            return -1;
        }

        //Check reprojection error
        cv::Point2f uv1 = this->project(x3D);

        float errX1 = uv1.x - kp1.pt.x;
        float errY1 = uv1.y - kp1.pt.y;

        if((errX1*errX1+errY1*errY1)>5.991 * sigmaLevel){   //Reprojection error is high
            return -1;
        }

        cv::Matx31f x3D2 = R21 * x3D + t21;
        cv::Point2f uv2 = pCamera2->project(x3D2);

        float errX2 = uv2.x - kp2.pt.x;
        float errY2 = uv2.y - kp2.pt.y;

        if((errX2*errX2+errY2*errY2)>5.991 * unc){   //Reprojection error is high
            return -1;
        }

        p3D = x3D;

        return z1;
    }

    std::ostream & operator<<(std::ostream &os, const KannalaBrandt8 &kb) {
        os << kb.mvParameters[0] << " " << kb.mvParameters[1] << " " << kb.mvParameters[2] << " " << kb.mvParameters[3] << " "
           << kb.mvParameters[4] << " " << kb.mvParameters[5] << " " << kb.mvParameters[6] << " " << kb.mvParameters[7];
        return os;
    }

    std::istream & operator>>(std::istream &is, KannalaBrandt8 &kb) {
        float nextParam;
        for(size_t i = 0; i < 8; i++){
            assert(is.good());  //Make sure the input stream is good
            is >> nextParam;
            kb.mvParameters[i] = nextParam;

        }
        return is;
    }

    void KannalaBrandt8::Triangulate(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Mat &Tcw1, const cv::Mat &Tcw2, cv::Mat &x3D)
    {
        cv::Mat A(4,4,CV_32F);

        A.row(0) = p1.x*Tcw1.row(2)-Tcw1.row(0);
        A.row(1) = p1.y*Tcw1.row(2)-Tcw1.row(1);
        A.row(2) = p2.x*Tcw2.row(2)-Tcw2.row(0);
        A.row(3) = p2.y*Tcw2.row(2)-Tcw2.row(1);

        cv::Mat u,w,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        x3D = vt.row(3).t();
        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
    }

    void KannalaBrandt8::Triangulate_(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Matx44f &Tcw1, const cv::Matx44f &Tcw2, cv::Matx31f &x3D)
    {
        cv::Matx14f A0, A1, A2, A3;


        A0 = p1.x * Tcw1.row(2) - Tcw1.row(0);
        A1 = p1.y * Tcw1.row(2) - Tcw1.row(1);
        A2 = p2.x * Tcw2.row(2) - Tcw2.row(0);
        A3 = p2.y * Tcw2.row(2) - Tcw2.row(1);
        cv::Matx44f A{A0(0), A0(1), A0(2), A0(3),
                      A1(0), A1(1), A1(2), A1(3),
                      A2(0), A2(1), A2(2), A2(3),
                      A3(0), A3(1), A3(2), A3(3)};

        // cv::Mat u,w,vt;
        cv::Matx44f u, vt;
        cv::Matx41f w;

        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        cv::Matx41f x3D_h = vt.row(3).t();
        x3D = x3D_h.get_minor<3,1>(0,0) / x3D_h(3);
    }
}

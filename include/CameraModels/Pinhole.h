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

#ifndef CAMERAMODELS_PINHOLE_H
#define CAMERAMODELS_PINHOLE_H

#include <assert.h>
#include <vector>
#include <opencv2/core/core.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include "GeometricCamera.h"

#include "TwoViewReconstruction.h"

namespace ORB_SLAM3 {
    class Pinhole : public GeometricCamera {

    public:
        Pinhole() {
            mvParameters.resize(4);
            mnId=nNextId++;
            mnType = CAM_PINHOLE;
        }
        Pinhole(const std::vector<float> _vParameters) : GeometricCamera(_vParameters), tvr(nullptr) {
            assert(mvParameters.size() == 4);
            mnId=nNextId++;
            mnType = CAM_PINHOLE;
        }

        Pinhole(Pinhole* pPinhole) : GeometricCamera(pPinhole->mvParameters), tvr(nullptr) {
            assert(mvParameters.size() == 4);
            mnId=nNextId++;
            mnType = CAM_PINHOLE;
        }


        ~Pinhole(){
            if(tvr) delete tvr;
        }

        cv::Point2f project(const cv::Point3f &p3D);
        cv::Point2f project(const cv::Matx31f &m3D);
        cv::Point2f project(const cv::Mat &m3D);
        Eigen::Vector2d project(const Eigen::Vector3d & v3D);
        cv::Mat projectMat(const cv::Point3f& p3D);

        float uncertainty2(const Eigen::Matrix<double,2,1> &p2D);

        cv::Point3f unproject(const cv::Point2f &p2D);
        cv::Mat unprojectMat(const cv::Point2f &p2D);
        cv::Matx31f unprojectMat_(const cv::Point2f &p2D);


        cv::Mat projectJac(const cv::Point3f &p3D);
        Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D);

        cv::Mat unprojectJac(const cv::Point2f &p2D);

        bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                                             cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);

        cv::Mat toK();
        cv::Matx33f toK_();

        bool epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::Mat& R12, const cv::Mat& t12, const float sigmaLevel, const float unc);
        bool epipolarConstrain_(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::Matx33f& R12, const cv::Matx31f& t12, const float sigmaLevel, const float unc);

        bool matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, GeometricCamera* pOther,
                                 cv::Mat& Tcw1, cv::Mat& Tcw2,
                                 const float sigmaLevel1, const float sigmaLevel2,
                                 cv::Mat& x3Dtriangulated) { return false;}

        friend std::ostream& operator<<(std::ostream& os, const Pinhole& ph);
        friend std::istream& operator>>(std::istream& os, Pinhole& ph);
    private:
        cv::Mat SkewSymmetricMatrix(const cv::Mat &v);
        cv::Matx33f SkewSymmetricMatrix_(const cv::Matx31f &v);

        //Parameters vector corresponds to
        //      [fx, fy, cx, cy]

        TwoViewReconstruction* tvr;
    };
}

//BOOST_CLASS_EXPORT_KEY(ORBSLAM2::Pinhole)

#endif //CAMERAMODELS_PINHOLE_H

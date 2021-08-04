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

#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <opencv2/core/core.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <Eigen/Geometry>

namespace ORB_SLAM3 {
    class GeometricCamera {

    public:
        GeometricCamera() {}
        GeometricCamera(const std::vector<float> &_vParameters) : mvParameters(_vParameters) {}
        ~GeometricCamera() {}

        virtual cv::Point2f project(const cv::Point3f &p3D) = 0;
        virtual cv::Point2f project(const cv::Matx31f &m3D) = 0;
        virtual cv::Point2f project(const cv::Mat& m3D) = 0;
        virtual Eigen::Vector2d project(const Eigen::Vector3d & v3D) = 0;
        virtual cv::Mat projectMat(const cv::Point3f& p3D) = 0;

        virtual float uncertainty2(const Eigen::Matrix<double,2,1> &p2D) = 0;

        virtual cv::Point3f unproject(const cv::Point2f &p2D) = 0;
        virtual cv::Mat unprojectMat(const cv::Point2f &p2D) = 0;
        virtual cv::Matx31f unprojectMat_(const cv::Point2f &p2D) = 0;

        virtual cv::Mat projectJac(const cv::Point3f &p3D) = 0;
        virtual Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D) = 0;

        virtual cv::Mat unprojectJac(const cv::Point2f &p2D) = 0;

        virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                                             cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated) = 0;

        virtual cv::Mat toK() = 0;
        virtual cv::Matx33f toK_() = 0;

        virtual bool epipolarConstrain(GeometricCamera* otherCamera, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::Mat& R12, const cv::Mat& t12, const float sigmaLevel, const float unc) = 0;
        virtual bool epipolarConstrain_(GeometricCamera* otherCamera, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::Matx33f& R12, const cv::Matx31f& t12, const float sigmaLevel, const float unc) = 0;

        float getParameter(const int i){return mvParameters[i];}
        void setParameter(const float p, const size_t i){mvParameters[i] = p;}

        size_t size(){return mvParameters.size();}

        virtual bool matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, GeometricCamera* pOther,
                                 cv::Mat& Tcw1, cv::Mat& Tcw2,
                                 const float sigmaLevel1, const float sigmaLevel2,
                                 cv::Mat& x3Dtriangulated) = 0;

        unsigned int GetId() { return mnId; }

        unsigned int GetType() { return mnType; }

        const unsigned int CAM_PINHOLE = 0;
        const unsigned int CAM_FISHEYE = 1;

        static long unsigned int nNextId;

    protected:
        std::vector<float> mvParameters;

        unsigned int mnId;

        unsigned int mnType;
    };
}


#endif //CAMERAMODELS_GEOMETRICCAMERA_H

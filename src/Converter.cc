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

#include "Converter.h"

namespace ORB_SLAM2
{
    std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
    {
        // 对一个大矩阵按行进行拆分
        // 在Frame中描述子是一整个大矩阵，而在KeyFrame中描述子是一个vector，每一个元素对应大矩阵的一行
        std::vector<cv::Mat> vDesc;
        vDesc.reserve(Descriptors.rows);
        for (int j = 0; j < Descriptors.rows; j++)
            vDesc.push_back(Descriptors.row(j));

        return vDesc;
    }

    g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
    {
        // 这个函数将传入的OpenCV的4*4的Mat类型矩阵（特殊欧式群SE3）转成g2o中的SE3Quat类型，便于使用
        // 首先新建了一个矩阵R用于存放旋转矩阵cvT中的旋转部分
        Eigen::Matrix<double, 3, 3> R;
        // 依次获取传入的变换矩阵中cvT的旋转部分
        R << cvT.at<float>(0, 0), cvT.at<float>(0, 1), cvT.at<float>(0, 2),
            cvT.at<float>(1, 0), cvT.at<float>(1, 1), cvT.at<float>(1, 2),
            cvT.at<float>(2, 0), cvT.at<float>(2, 1), cvT.at<float>(2, 2);
        // 再新建一个3*1的向量，并将对应平移的部分传入
        Eigen::Matrix<double, 3, 1> t(cvT.at<float>(0, 3), cvT.at<float>(1, 3), cvT.at<float>(2, 3));

        // 返回根据R、t构建的g2o的SE3Quat对象
        return g2o::SE3Quat(R, t);
    }

    cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
    {
        // 调用g2o的SE3的成员函数to_homogeneous_matrix就将数据转换成了Eigen的Matrix格式
        Eigen::Matrix<double, 4, 4> eigMat = SE3.to_homogeneous_matrix();
        // 然后再将Eigen的Matrix转换成OpenCV的Mat
        return toCvMat(eigMat);
    }

    cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
    {
        // 将g2o的Sim3转换为Eigen的旋转矩阵、平移向量和尺度因子
        // 然后调用toCvSE3将尺度因子*旋转矩阵，平移向量转换为OpenCV的4*4的Mat
        Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = Sim3.translation();
        double s = Sim3.scale();
        return toCvSE3(s * eigR, eigt);
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 4, 4> &m)
    {
        // 将eigen的4*4的Matrix转换为OpenCV的Mat
        cv::Mat cvMat(4, 4, CV_32F);
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                cvMat.at<float>(i, j) = m(i, j);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
    {
        // 将eigen的3*3的Matrix转换为OpenCV的Mat
        cv::Mat cvMat(3, 3, CV_32F);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cvMat.at<float>(i, j) = m(i, j);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 3, 1> &m)
    {
        // 将eigen的3*1的Matrix转换为OpenCV的3*1的Mat
        cv::Mat cvMat(3, 1, CV_32F);
        for (int i = 0; i < 3; i++)
            cvMat.at<float>(i) = m(i);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvSE3(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &t)
    {
        // 将eigen的3*3和3*1的Matrix转换为OpenCV的4*4的Mat
        cv::Mat cvMat = cv::Mat::eye(4, 4, CV_32F);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                cvMat.at<float>(i, j) = R(i, j);
            }
        }
        for (int i = 0; i < 3; i++)
        {
            cvMat.at<float>(i, 3) = t(i);
        }

        return cvMat.clone();
    }

    Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Mat &cvVector)
    {
        // 将OpenCV的Mat转换为eigen的3*1的Matrix
        Eigen::Matrix<double, 3, 1> v;
        v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

        return v;
    }

    Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Point3f &cvPoint)
    {
        // 将OpenCV的Point3f转换为eigen的3*1的Matrix
        Eigen::Matrix<double, 3, 1> v;
        v << cvPoint.x, cvPoint.y, cvPoint.z;

        return v;
    }

    Eigen::Matrix<double, 3, 3> Converter::toMatrix3d(const cv::Mat &cvMat3)
    {
        // 将OpenCV的Mat转换为eigen的3*3的Matrix
        Eigen::Matrix<double, 3, 3> M;

        M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
            cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
            cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

        return M;
    }

    std::vector<float> Converter::toQuaternion(const cv::Mat &M)
    {
        // 将输入的旋转矩阵转换成对应的vector类型的四元数表示
        // 具体来说，首先第一步，利用函数toMatrix3d将OpenCV的Mat转换成了Eigen的Matrix3d
        Eigen::Matrix<double, 3, 3> eigMat = toMatrix3d(M);
        // 然后得到旋转矩阵，直接用Eigen的API，将旋转矩阵转换成了四元数q
        Eigen::Quaterniond q(eigMat);

        // 最后为了使用方便，转换为vector的形式，四元数的形式为qx,qy,qz,qw
        std::vector<float> v(4);
        v[0] = q.x();
        v[1] = q.y();
        v[2] = q.z();
        v[3] = q.w();

        return v;
    }

} // namespace ORB_SLAM

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
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
//#include <line_descriptor/descriptor_custom.hpp>
//#include <line_descriptor_custom.hpp>

#include "Frame.h"
#include "auxiliar.h"


namespace ORB_SLAM2
{

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
    /**
     * @brief 单目SLAM初始化相关，双目和RGBD不会使用这个类
     */
    class Initializer
    {
        typedef pair<int,int> Match;

    public:

        // Fix the reference frame
        // 用reference frame来初始化，这个reference frame就是SLAM正式开始的第一帧
        Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

        // Computes in parallel a fundamental matrix and a homography
        // Selects a model and tries to recover the motion and the structure from motion
        // 用current frame，也就是用SLAM逻辑上的第二帧来初始化整个SLAM，得到最开始两帧之间的R，t，以及点云
        bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                        cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);

        // 函数重载的初始化函数，包括点特征和线特征, R21和t21均为第一帧到第二帧的变换
        bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                        cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated,
                        vector<pair<int,int>> &vLineMatches, vector<cv::Point3f> &vLineS3D, vector<cv::Point3f> &vLineE3D,
                        vector<bool> &vbLineTriangulated);

        bool InitializeManhattanWorld(const Frame &mInitialFrame,const Frame &CurrentFrame, const vector<int> &vMatches12,
                                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated,
                                      vector<pair<int,int>> &vLineMatches, vector<cv::Point3f> &vLineS3D, vector<cv::Point3f> &vLineE3D,
                                      vector<bool> &vbLineTriangulated);


    private:

        // 假设场景为平面情况下，通过前两帧求取Homography矩阵（current frame 2 到reference frame 1），并得到该模型的得分
        void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
        // 假设场景为非平面情况下，通过前两帧求Fundamental矩阵,并得到该模型的得分
        void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

        cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
        cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

        float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

        float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

        //重建3D点
        bool Reconstruct(vector<bool> &vbMatchesInliers,  cv::Mat &K,
                         cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        // 分解F矩阵，并从分解后的多个解中找出合适的R，t
        bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        // 分解H矩阵，并从分解后的多个解中找出合适的R，t
        bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        // 三角化，利用反投影矩阵将特征点恢复为3D点
        void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

        // 归一化三维空间点和帧间位移t
        void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

        // ReconstructF调用该函数进行检测，从而进一步找出F分解后最合适的解
        int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                    const vector<Match> &vMatches12, vector<bool> &vbInliers,
                    const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

        // 由F矩阵分解求得Essential矩阵, F=K.inverse().transpose()*E*K.inverse()
        void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

        // 线特征的三角化
        // 仿照点特征，采用两帧匹配的特征线段的端点
        void LineTriangulate(const KeyLine &kl1, const KeyLine &kl2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &LineStart3D, cv::Mat &LineEnd3D);
        // 结合极平面
        void LineTriangulate(const KeyLine &kl1, const KeyLine &kl2, const cv::Mat &P1, const cv::Mat &P2, const Vector3d &klf1, const Vector3d &klf2,
                             cv::Mat &LineStart3D, cv::Mat &LineEnd3D);

        // 用计算好的R t，来三角化计算线特征
        void ReconstructLine(vector<Match> &vLineMatchesH, cv::Mat &K, cv::Mat &R21, cv::Mat &t21,
                             vector<Vector3d> &vKeyLineFunctions1, vector<Vector3d> &vKeyLineFunctions2,
                             vector<Point3f> &vLineS3D, vector<Point3f> &vLineE3D, vector<bool> &vinliers);

        // 存储参考帧的线特征集 Frame1，以及线特征所在直线的集合
        vector<KeyLine> mvKeyLines1;
        vector<Vector3d> mvKeyLineFunctions1;

        // 存储当前帧的线特征集 Frame2，以及线特征所在直线的集合
        vector<KeyLine> mvKeyLines2;
        vector<Vector3d> mvKeyLineFunctions2;

        // 从参考帧到当前帧的线匹配对
        vector<Match> mvLineMatches12;

        // 记录reference frame的每个线特征在Current frame中是否有匹配对
        vector<bool> mvbLineMatched1;

        // Keypoints from Reference Frame (Frame 1)
        vector<cv::KeyPoint> mvKeys1;   ///< 存储参考帧的特征点集

        // Keypoints from Current Frame (Frame 2)
        vector<cv::KeyPoint> mvKeys2;   ///< 存储当前帧的特征点集

        // Current Matches from Reference to Current
        vector<Match> mvMatches12;  /// Match的数据结构是pair，mvMatches12只记录Reference到Current匹配上的特征点对
        vector<bool> mvbMatched1;   /// 记录Reference Frame的每个特征点在Current Frame是否有匹配的特征点

        // Calibration
        cv::Mat mK;     /// 相机内参

        // Standard Deviation and Variance
        float mSigma, mSigma2;  /// 测量误差

        // Ransac max iterations
        int mMaxIterations;     /// 计算基础矩阵和单应矩阵时，会用到RANSAC去除异常值，该值最最大迭代次数

        // Ransac sets
        vector<vector<size_t> > mvSets;   ///< 外层容器的大小为迭代次数，内层容器大小为每次迭代计算H或F矩阵需要的点

    };

} //namespace ORB_SLAM

#endif // INITIALIZER_H

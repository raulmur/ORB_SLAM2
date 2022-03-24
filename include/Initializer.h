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

#include <opencv2/opencv.hpp>
#include "Frame.h"

// See ~ ~ ~ ~ ~ ~ ~ ~
// https://blog.csdn.net/xiaoxiaowenqiang/article/details/79278884
// https://www.jianshu.com/p/479b0e7bd99e

namespace ORB_SLAM2
{

    // THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
    class Initializer
    {
        typedef pair<int, int> Match;

    public:
        // Fix the reference frame
        // 固定参考帧
        Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

        // Computes in parallel a fundamental matrix and a homography
        // Selects a model and tries to recover the motion and the structure from motion
        bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                        cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);

    private:
        void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
        void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

        cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
        cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

        float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

        float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

        bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

        void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

        int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                    const vector<Match> &vMatches12, vector<bool> &vbInliers,
                    const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

        void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

        // Keypoints from Reference Frame (Frame 1) 参考帧中的特征点
        vector<cv::KeyPoint> mvKeys1;

        // Keypoints from Current Frame (Frame 2)   当前帧中的特征点
        vector<cv::KeyPoint> mvKeys2;

        // Current Matches from Reference to Current
        // 从参考帧到当前帧的特征点匹配关系，匹配关系实际上是vector之间索引的对应关系
        // 这里匹配时通过pair类型实现，pair由两个int数构成，分别保存的是匹配点对在mvKeys1和mvKeys2中的索引
        vector<Match> mvMatches12;
        vector<bool> mvbMatched1;   // 它是用来统计RANSAC算法内点个数的变量；内点的含义是RANSAC中的概念
        
        // RANSAC基础版的步骤（RANSAC是一种概率算法）
        // 1. 在原始数据中随机选取一个（最小）子集作为假设内点
        // 2. 根据假设的内点拟合一个模型
        // 3. 判断剩余的原始数据是否符合你和的模型，将其分为内点和外点。如果内点过少则标记为无效迭代
        // 4. 根据假设的内点和上一步划分的内点重新拟合模型
        // 5. 计算所有内点的残差，根据残差和或者错误率重新评估模型
        // 迭代以上步骤，把具有最小残差和或是最多内点数的模型作为最佳模型
        // 最小二乘法尽量去适应包括外点在内的所有点。而RANSAC得出一个仅仅用内点计算出来的模型，并且概率还足够高。但是RANSAC不能保证结果一定正确（只有一定的概率得到可信的模型），为了保证算法有足够高的合理概率，必须小心选择算法的参数。对于外点数量较多的数据集，RANSAC的效果远优于直接的最小二乘法。

        // Calibration
        cv::Mat mK;

        // Standard Deviation and Variance
        float mSigma, mSigma2;

        // Ransac max iterations    RANSAC最大迭代次数
        int mMaxIterations;

        // Ransac sets  RANSAC的选择集
        // 外面的vector长度等于匹配点对的个数，里面的vector长度为计算H的个数8，里面的元素内容是某点在mvMatches12中的索引
        // mvMatches12中存放的是自定义的Match类型（本质是pair类型，可以通过first和second分别获得两个数）
        // Match类型由两个int构成，分别存放的是匹配点对在mvKeys1和mvKeys2中的索引
        vector<vector<size_t>> mvSets;
    };

} // namespace ORB_SLAM

#endif // INITIALIZER_H

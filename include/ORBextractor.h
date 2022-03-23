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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>

namespace ORB_SLAM2
{

    class ExtractorNode
    {
    public:
        ExtractorNode() : bNoMore(false) {}

        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        bool bNoMore;
    };

    class ORBextractor
    {
    public:
        enum
        {
            HARRIS_SCORE = 0,
            FAST_SCORE = 1
        };

        ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                     int iniThFAST, int minThFAST);

        ~ORBextractor() {}

        // Compute the ORB features and descriptors on an image.
        // ORB are dispersed on the image using an octree.
        // Mask is ignored in the current implementation.
        // 重载了()运算符，方便直接调用
        // 这样新建了一个对象之后就可以像调用函数一样调用它了
        // 这也是对外执行ORB特征提取的唯一接口
        void operator()(cv::InputArray image, cv::InputArray mask,
                        std::vector<cv::KeyPoint> &keypoints,
                        cv::OutputArray descriptors);

        int inline GetLevels()
        {
            return nlevels;
        }

        float inline GetScaleFactor()
        {
            return scaleFactor;
        }

        std::vector<float> inline GetScaleFactors()
        {
            return mvScaleFactor;
        }

        std::vector<float> inline GetInverseScaleFactors()
        {
            return mvInvScaleFactor;
        }

        std::vector<float> inline GetScaleSigmaSquares()
        {
            return mvLevelSigma2;
        }

        std::vector<float> inline GetInverseScaleSigmaSquares()
        {
            return mvInvLevelSigma2;
        }

        // 图像金字塔，存放每一层图像的内容
        std::vector<cv::Mat> mvImagePyramid;

    protected:
        void ComputePyramid(cv::Mat image);
        void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);
        std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                                    const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

        void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);
        std::vector<cv::Point> pattern;

        int nfeatures;      // 特征点个数
        double scaleFactor; // 金字塔的比例因子
        int nlevels;        // 金字塔的层数
        int iniThFAST;      // 初始FAST阈值
        int minThFAST;      // 最小FAST阈值

        // 用于存储金字塔每一层的特征点的个数
        std::vector<int> mnFeaturesPerLevel;

        std::vector<int> umax;

        // 多个vector, 用于储存每层的尺度因子和逆尺度信息
        std::vector<float> mvScaleFactor;
        std::vector<float> mvInvScaleFactor;
        std::vector<float> mvLevelSigma2;
        std::vector<float> mvInvLevelSigma2;
    };

} // namespace ORB_SLAM

#endif

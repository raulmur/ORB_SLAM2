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

#include "Initializer.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include "Optimizer.h"
#include "ORBmatcher.h"

#include <thread>

namespace ORB_SLAM2
{
    // 将参考帧的内参矩阵K和校正过的特征点坐标对应赋给Initializer的成员变量
    Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
    {
        mK = ReferenceFrame.mK.clone();

        mvKeys1 = ReferenceFrame.mvKeysUn;

        mSigma = sigma;
        mSigma2 = sigma * sigma;
        mMaxIterations = iterations;
    }

    bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                                 vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
    {
        // Fill structures with current keypoints and matches with reference frame
        // Reference Frame: 1, Current Frame: 2
        mvKeys2 = CurrentFrame.mvKeysUn;

        mvMatches12.clear();
        mvMatches12.reserve(mvKeys2.size());
        mvbMatched1.resize(mvKeys1.size());
        for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
        {
            if (vMatches12[i] >= 0)
            {
                mvMatches12.push_back(make_pair(i, vMatches12[i]));
                mvbMatched1[i] = true;
            }
            else
                mvbMatched1[i] = false;
        }

        const int N = mvMatches12.size();

        // Indices for minimum set selection
        vector<size_t> vAllIndices;
        vAllIndices.reserve(N);
        vector<size_t> vAvailableIndices;

        // 将每个元素对应的索引作为内容存进来
        for (int i = 0; i < N; i++)
        {
            vAllIndices.push_back(i);
        }

        // Generate sets of 8 points for each RANSAC iteration
        // 按照迭代次数新建一个vector，每一个元素又都是包含8个元素的vector，这8个元素就是我们要随机选择的用于RANSAC的点
        mvSets = vector<vector<size_t>>(mMaxIterations, vector<size_t>(8, 0));

        // 下面的for循环都是在迭代选择点
        DUtils::Random::SeedRandOnce(0);

        // 这里的随机选择策略是保证每次迭代选择的8个点不会重复，本轮迭代中如果点选过了下次就不会再选了，不同迭代次数之间选择的点可以重复
        for (int it = 0; it < mMaxIterations; it++)
        {
            // 每次迭代中都将全部的点赋给一个临时变量便于选择点
            vAvailableIndices = vAllIndices;

            // Select a minimum set
            for (size_t j = 0; j < 8; j++)
            {
                // 生成一个随机数
                int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);
                // 获取对应内容
                int idx = vAvailableIndices[randi];
                // 将值赋给8个元素中的对应元素
                mvSets[it][j] = idx;

                // 选一个元素就八这个元素丢掉，这样下次就不会再选了
                // back函数返回当前vector容器中末尾元素的引用
                vAvailableIndices[randi] = vAvailableIndices.back();
                // 将vector的最后一个元素删除
                vAvailableIndices.pop_back();
            }
        }

        // Launch threads to compute in parallel a fundamental matrix and a homography
        vector<bool> vbMatchesInliersH, vbMatchesInliersF; // 得分最高的H和F时匹配的内点情况
        float SH, SF;                                      // H的得分、F的得分
        cv::Mat H, F;                                      // 单应矩阵、基础矩阵

        // 开多个线程分别计算H和F，计算的核心函数是FindHomography和FindFundamental
        // 对于这两个函数，都会返回利用RANSAC经过一定迭代次数后的最好的变换关系以及其对应的得分
        thread threadH(&Initializer::FindHomography, this, ref(vbMatchesInliersH), ref(SH), ref(H));
        thread threadF(&Initializer::FindFundamental, this, ref(vbMatchesInliersF), ref(SF), ref(F));

        // Wait until both threads have finished
        // 等待线程同步
        threadH.join();
        threadF.join();

        // Compute ratio of scores
        // 作者自定义的H和F的选择指标
        float RH = SH / (SH + SF);

        // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
        // 如果RH待遇0.4，使用单应矩阵作为估计，否则使用基础矩阵
        // 其实到这一步初始化就差不多成功了，剩下的就是根据求出来的变换关系（H或者F）分解得到R，t（当然包含多种解的过滤），最后用R，t对特征点进行三角化
        if (RH > 0.40)
            // 根据单应矩阵H恢复R、t以及三角化特征点
            // 注意这个函数里的输入输出参数
            // 首先是vbMatchesInliersH，它是在估计的最好H的情况下，内点的匹配情况（true or false）
            // 然后传入的是求解的H矩阵以及成员变量mK(相机的内参矩阵)，它在Initializer构造函数的第一行就被赋值了
            // 然后还有最后两个运行参数，对应checkRT函数中的阈值，帧间视差（度）、最少成功三角化的特征点，用于控制三角化点是否可靠
            // 返回值是分解H得到的R、t、三角化后的三维点，以及哪些三维点是可靠的
            return ReconstructH(vbMatchesInliersH, H, mK, R21, t21, vP3D, vbTriangulated, 1.0, 50);
        else // if(pF_HF>0.6)
            return ReconstructF(vbMatchesInliersF, F, mK, R21, t21, vP3D, vbTriangulated, 1.0, 50);

        return false;
    }

    void Initializer::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
    {
        // Number of putative（认定的） matches
        const int N = mvMatches12.size();

        // Normalize coordinates
        vector<cv::Point2f> vPn1, vPn2;
        cv::Mat T1, T2;
        // 对特征点坐标进行归一化，并获得归一化矩阵，它代表的是原始坐标到归一化坐标的变换
        Normalize(mvKeys1, vPn1, T1);
        Normalize(mvKeys2, vPn2, T2);
        cv::Mat T2inv = T2.inv();

        // Best Results variables   // 这两个变量用来存储最好的结果
        score = 0.0; // 单应矩阵的得分，默认为0
        // vbMatchesInliers的长度和mvMatches12相等，表示的是一共匹配到了多少点对
        // 而至于是不是内点，则是根据它其中每个元素的true or false判断的
        // 通过统计vbMatchesInliers中true的个数，即可以算出内点的个数，进而可以算出得分
        vbMatchesInliers = vector<bool>(N, false);

        // Iteration variables  这些变量用于迭代
        vector<cv::Point2f> vPn1i(8);            // 长度为8的临时变量，用于存放随机选取的第一帧中的8对点
        vector<cv::Point2f> vPn2i(8);            // 长度为8的临时变量，用于存放随机选取的第二帧中的8对点
        cv::Mat H21i, H12i;                      // 从2到1和从1到2的单应矩阵
        vector<bool> vbCurrentInliers(N, false); // 用于存放有多少是内点的vector，如果内点即为true，否则为false
        float currentScore;

        // Perform all RANSAC iterations and save the solution with highest score
        for (int it = 0; it < mMaxIterations; it++)
        {
            // Select a minimum set
            // 由于采用的是RANSAC算法，所以每次迭代随机选8对点，这些点对在Initializer函数中已经选过了，这里直接根据索引读取内容
            // 由于由8个点，所以一次读取8次
            for (size_t j = 0; j < 8; j++)
            {
                // 这里再整理一下思路，之前已经说过了mvSet的每一个元素代表一次迭代，存放的是随机选取的8对点再mvMatches12中的索引
                // 而mvMatches中存放的则是该匹配点在mvKeys1和mvKeys2中的索引，所以下面的代码就清晰了

                // 这里的idx表示的就是第it次迭代，选区的第j对点在mvMatches12中的索引
                int idx = mvSets[it][j];

                // 由于刚刚已经介绍了mvMatches12的存储内容，所以我们可以直接通过索引获取到第idx点对应的特征点在mvKeys1和mvKeys2中的索引
                // 根据这个索引就可以很容易地获取那个特征点地真实的对象了
                // 需要注意的是，这里使用归一化后的坐标，所以索引索取的是vPn1和vPn2，它们和mvKeys1和mvKeys2是一一对应的
                // pair类型有first和second属性，用于获取对应数据
                vPn1i[j] = vPn1[mvMatches12[idx].first];
                vPn2i[j] = vPn2[mvMatches12[idx].second];
            }

            // 计算H的步骤这里进行，主义的是参数输入顺序是第一帧的点对、第二帧的点对，返回的H是从2到1的单应矩阵
            // 另一个需要注意的是传入的是归一化后的8对对应坐标
            cv::Mat Hn = ComputeH21(vPn1i, vPn2i);
            // 由于刚刚我们对特征点坐标进行了正则化，所以这里恢复成原来的坐标，这样这里的H表示的就是原始坐标间的变换关系了
            H21i = T2inv * Hn * T1;
            H12i = H21i.inv();

            // 计算当前计算得到的H的得分，简单来说是统计内点的多少，越多得分越高
            currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

            // 如果说当前的单应矩阵算出的得分比上一次的好，就将当前的H返回
            // 随着迭代次数不断增加，结果只会越来越好
            if (currentScore > score)
            {
                H21 = H21i.clone();
                vbMatchesInliers = vbCurrentInliers;
                score = currentScore;
            }
        }
    }

    void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
    {
        // Number of putative matches
        const int N = vbMatchesInliers.size();

        // Normalize coordinates
        vector<cv::Point2f> vPn1, vPn2;
        cv::Mat T1, T2;
        Normalize(mvKeys1, vPn1, T1);
        Normalize(mvKeys2, vPn2, T2);
        cv::Mat T2t = T2.t();

        // Best Results variables
        score = 0.0;
        vbMatchesInliers = vector<bool>(N, false);

        // Iteration variables
        vector<cv::Point2f> vPn1i(8);
        vector<cv::Point2f> vPn2i(8);
        cv::Mat F21i;
        vector<bool> vbCurrentInliers(N, false);
        float currentScore;

        // Perform all RANSAC iterations and save the solution with highest score
        for (int it = 0; it < mMaxIterations; it++)
        {
            // Select a minimum set
            for (int j = 0; j < 8; j++)
            {
                int idx = mvSets[it][j];

                vPn1i[j] = vPn1[mvMatches12[idx].first];
                vPn2i[j] = vPn2[mvMatches12[idx].second];
            }

            cv::Mat Fn = ComputeF21(vPn1i, vPn2i);

            F21i = T2t * Fn * T1;

            currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

            if (currentScore > score)
            {
                F21 = F21i.clone();
                vbMatchesInliers = vbCurrentInliers;
                score = currentScore;
            }
        }
    }

    cv::Mat Initializer::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
    {
        // 计算H的步骤就是用匹配点对构造系数矩阵A
        // 然后对A进行SVD分解，分解结果的V最后一列就是要求的H
        const int N = vP1.size();

        cv::Mat A(2 * N, 9, CV_32F);

        // 构造系数矩阵A
        for (int i = 0; i < N; i++)
        {
            const float u1 = vP1[i].x;
            const float v1 = vP1[i].y;
            const float u2 = vP2[i].x;
            const float v2 = vP2[i].y;

            A.at<float>(2 * i, 0) = 0.0;
            A.at<float>(2 * i, 1) = 0.0;
            A.at<float>(2 * i, 2) = 0.0;
            A.at<float>(2 * i, 3) = -u1;
            A.at<float>(2 * i, 4) = -v1;
            A.at<float>(2 * i, 5) = -1;
            A.at<float>(2 * i, 6) = v2 * u1;
            A.at<float>(2 * i, 7) = v2 * v1;
            A.at<float>(2 * i, 8) = v2;

            A.at<float>(2 * i + 1, 0) = u1;
            A.at<float>(2 * i + 1, 1) = v1;
            A.at<float>(2 * i + 1, 2) = 1;
            A.at<float>(2 * i + 1, 3) = 0.0;
            A.at<float>(2 * i + 1, 4) = 0.0;
            A.at<float>(2 * i + 1, 5) = 0.0;
            A.at<float>(2 * i + 1, 6) = -u2 * u1;
            A.at<float>(2 * i + 1, 7) = -u2 * v1;
            A.at<float>(2 * i + 1, 8) = -u2;
        }

        cv::Mat u, w, vt;
        // 调用OpenCV的API进行SVD分解
        cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
        // SVD分解结果的V矩阵的最后一列
        // 注意的是这里得到的是VT，所以也就是对应VT的最后一行
        return vt.row(8).reshape(0, 3);
    }

    cv::Mat Initializer::ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
    {
        const int N = vP1.size();

        cv::Mat A(N, 9, CV_32F);

        for (int i = 0; i < N; i++)
        {
            const float u1 = vP1[i].x;
            const float v1 = vP1[i].y;
            const float u2 = vP2[i].x;
            const float v2 = vP2[i].y;

            A.at<float>(i, 0) = u2 * u1;
            A.at<float>(i, 1) = u2 * v1;
            A.at<float>(i, 2) = u2;
            A.at<float>(i, 3) = v2 * u1;
            A.at<float>(i, 4) = v2 * v1;
            A.at<float>(i, 5) = v2;
            A.at<float>(i, 6) = u1;
            A.at<float>(i, 7) = v1;
            A.at<float>(i, 8) = 1;
        }

        cv::Mat u, w, vt;

        cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        cv::Mat Fpre = vt.row(8).reshape(0, 3);

        cv::SVDecomp(Fpre, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        w.at<float>(2) = 0;

        return u * cv::Mat::diag(w) * vt;
    }

    float Initializer::CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
    {
        // 根据求解得到的单应矩阵计算哪些匹配点对是内点，哪些是外点，并给出评分
        const int N = mvMatches12.size();

        const float h11 = H21.at<float>(0, 0);
        const float h12 = H21.at<float>(0, 1);
        const float h13 = H21.at<float>(0, 2);
        const float h21 = H21.at<float>(1, 0);
        const float h22 = H21.at<float>(1, 1);
        const float h23 = H21.at<float>(1, 2);
        const float h31 = H21.at<float>(2, 0);
        const float h32 = H21.at<float>(2, 1);
        const float h33 = H21.at<float>(2, 2);

        const float h11inv = H12.at<float>(0, 0);
        const float h12inv = H12.at<float>(0, 1);
        const float h13inv = H12.at<float>(0, 2);
        const float h21inv = H12.at<float>(1, 0);
        const float h22inv = H12.at<float>(1, 1);
        const float h23inv = H12.at<float>(1, 2);
        const float h31inv = H12.at<float>(2, 0);
        const float h32inv = H12.at<float>(2, 1);
        const float h33inv = H12.at<float>(2, 2);

        vbMatchesInliers.resize(N);

        // 对应关系的得分
        float score = 0;

        // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
        const float th = 5.991;

        // 信息矩阵，方差平方的倒数
        const float invSigmaSquare = 1.0 / (sigma * sigma);

        // 对于每一对点，会正反计算两个误差，对应两个得分
        for (int i = 0; i < N; i++)
        {
            bool bIn = true;

            // 先获取到对应点在各自图像上的像素坐标
            const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
            const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

            const float u1 = kp1.pt.x;
            const float v1 = kp1.pt.y;
            const float u2 = kp2.pt.x;
            const float v2 = kp2.pt.y;

            // Reprojection error in first image
            // x2in1 = H12*x2
            // 计算第一帧中的重投影误差：将第二帧中的点按照算出来的H投影到第一帧上并于真值比较
            // 这里的公式并不复杂，将上面的矩阵形式写开就可以得到了，因为是齐次坐标，最后一维需要为1，所以同时除以w2in1inv
            const float w2in1inv = 1.0 / (h31inv * u2 + h32inv * v2 + h33inv);
            const float u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
            const float v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;

            // 计算重投影后两个像素点的距离作为重投影误差
            const float squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);

            // 得到重投影误差后又乘了invSigmaSquare，以此结果作为判断依据
            // 根据方差归一化误差
            // 这个地方需要注意一下！！！
            const float chiSquare1 = squareDist1 * invSigmaSquare;

            // 如果它大于阈值，认为该点是外点，设为false
            // 否则的话，将它与阈值的差值作为得分
            if (chiSquare1 > th)
                bIn = false; // 在循环的一开始设置了为true，所以如果是内点就不用再做什么操作了
            else
                score += th - chiSquare1;

            // 上面计算的是第二帧上的特征点根据变换关系投影到第一帧上的误差和得分，这还不算完，还可以反着算一遍
            // 算法核心思想和流程和上面是一样的
            // Reprojection error in second image
            // x1in2 = H21*x1
            // 计算第二帧中的重投影误差：将第一帧中的点按照算出来的H投影到第二帧上并与真值比较
            const float w1in2inv = 1.0 / (h31 * u1 + h32 * v1 + h33);
            const float u1in2 = (h11 * u1 + h12 * v1 + h13) * w1in2inv;
            const float v1in2 = (h21 * u1 + h22 * v1 + h23) * w1in2inv;

            const float squareDist2 = (u2 - u1in2) * (u2 - u1in2) + (v2 - v1in2) * (v2 - v1in2);

            const float chiSquare2 = squareDist2 * invSigmaSquare;

            if (chiSquare2 > th)
                bIn = false;
            else
                score += th - chiSquare2;

            // 只有当在两帧中的重投影误差都在阈值范围内，才任务该点是内点，设为true，否则为false
            if (bIn)
                vbMatchesInliers[i] = true;
            else
                vbMatchesInliers[i] = false;
        }

        // 每一个内点点对都可以算出来两个得分（1到2一个，2到1一个），这些得分求和之后作为这个单应矩阵的得分返回
        return score;
    }

    float Initializer::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
    {
        const int N = mvMatches12.size();

        const float f11 = F21.at<float>(0, 0);
        const float f12 = F21.at<float>(0, 1);
        const float f13 = F21.at<float>(0, 2);
        const float f21 = F21.at<float>(1, 0);
        const float f22 = F21.at<float>(1, 1);
        const float f23 = F21.at<float>(1, 2);
        const float f31 = F21.at<float>(2, 0);
        const float f32 = F21.at<float>(2, 1);
        const float f33 = F21.at<float>(2, 2);

        vbMatchesInliers.resize(N);

        float score = 0;

        const float th = 3.841;
        const float thScore = 5.991;

        const float invSigmaSquare = 1.0 / (sigma * sigma);

        for (int i = 0; i < N; i++)
        {
            bool bIn = true;

            const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
            const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

            const float u1 = kp1.pt.x;
            const float v1 = kp1.pt.y;
            const float u2 = kp2.pt.x;
            const float v2 = kp2.pt.y;

            // Reprojection error in second image
            // l2=F21x1=(a2,b2,c2)

            const float a2 = f11 * u1 + f12 * v1 + f13;
            const float b2 = f21 * u1 + f22 * v1 + f23;
            const float c2 = f31 * u1 + f32 * v1 + f33;

            const float num2 = a2 * u2 + b2 * v2 + c2;

            const float squareDist1 = num2 * num2 / (a2 * a2 + b2 * b2);

            const float chiSquare1 = squareDist1 * invSigmaSquare;

            if (chiSquare1 > th)
                bIn = false;
            else
                score += thScore - chiSquare1;

            // Reprojection error in second image
            // l1 =x2tF21=(a1,b1,c1)

            const float a1 = f11 * u2 + f21 * v2 + f31;
            const float b1 = f12 * u2 + f22 * v2 + f32;
            const float c1 = f13 * u2 + f23 * v2 + f33;

            const float num1 = a1 * u1 + b1 * v1 + c1;

            const float squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);

            const float chiSquare2 = squareDist2 * invSigmaSquare;

            if (chiSquare2 > th)
                bIn = false;
            else
                score += thScore - chiSquare2;

            if (bIn)
                vbMatchesInliers[i] = true;
            else
                vbMatchesInliers[i] = false;
        }

        return score;
    }

    bool Initializer::ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                                   cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
    {
        int N = 0;
        for (size_t i = 0, iend = vbMatchesInliers.size(); i < iend; i++)
            if (vbMatchesInliers[i])
                N++;

        // Compute Essential Matrix from Fundamental Matrix
        cv::Mat E21 = K.t() * F21 * K;

        cv::Mat R1, R2, t;

        // Recover the 4 motion hypotheses
        DecomposeE(E21, R1, R2, t);

        cv::Mat t1 = t;
        cv::Mat t2 = -t;

        // Reconstruct with the 4 hyphoteses and check
        vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
        vector<bool> vbTriangulated1, vbTriangulated2, vbTriangulated3, vbTriangulated4;
        float parallax1, parallax2, parallax3, parallax4;

        int nGood1 = CheckRT(R1, t1, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D1, 4.0 * mSigma2, vbTriangulated1, parallax1);
        int nGood2 = CheckRT(R2, t1, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D2, 4.0 * mSigma2, vbTriangulated2, parallax2);
        int nGood3 = CheckRT(R1, t2, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D3, 4.0 * mSigma2, vbTriangulated3, parallax3);
        int nGood4 = CheckRT(R2, t2, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D4, 4.0 * mSigma2, vbTriangulated4, parallax4);

        int maxGood = max(nGood1, max(nGood2, max(nGood3, nGood4)));

        R21 = cv::Mat();
        t21 = cv::Mat();

        int nMinGood = max(static_cast<int>(0.9 * N), minTriangulated);

        int nsimilar = 0;
        if (nGood1 > 0.7 * maxGood)
            nsimilar++;
        if (nGood2 > 0.7 * maxGood)
            nsimilar++;
        if (nGood3 > 0.7 * maxGood)
            nsimilar++;
        if (nGood4 > 0.7 * maxGood)
            nsimilar++;

        // If there is not a clear winner or not enough triangulated points reject initialization
        if (maxGood < nMinGood || nsimilar > 1)
        {
            return false;
        }

        // If best reconstruction has enough parallax initialize
        if (maxGood == nGood1)
        {
            if (parallax1 > minParallax)
            {
                vP3D = vP3D1;
                vbTriangulated = vbTriangulated1;

                R1.copyTo(R21);
                t1.copyTo(t21);
                return true;
            }
        }
        else if (maxGood == nGood2)
        {
            if (parallax2 > minParallax)
            {
                vP3D = vP3D2;
                vbTriangulated = vbTriangulated2;

                R2.copyTo(R21);
                t1.copyTo(t21);
                return true;
            }
        }
        else if (maxGood == nGood3)
        {
            if (parallax3 > minParallax)
            {
                vP3D = vP3D3;
                vbTriangulated = vbTriangulated3;

                R1.copyTo(R21);
                t2.copyTo(t21);
                return true;
            }
        }
        else if (maxGood == nGood4)
        {
            if (parallax4 > minParallax)
            {
                vP3D = vP3D4;
                vbTriangulated = vbTriangulated4;

                R2.copyTo(R21);
                t2.copyTo(t21);
                return true;
            }
        }

        return false;
    }

    bool Initializer::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                                   cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
    {
        int N = 0;
        // 一个小技巧，如果在for循环里写int，然后循环到vector.size()结束的话，编译可能回出现类型不一致警告（这是因为vector.size()默认类型为size_t，使用int可能会超出限制）
        // 因为循环变量是int而size()函数返回的是unsigned int
        // 这里传入的变量vbMatchesInliers和匹配到的点对个数是一样的
        // 但这里又进行了一次筛选，只选取了满足单应矩阵H变换的匹配点对
        for (size_t i = 0, iend = vbMatchesInliers.size(); i < iend; i++)
            if (vbMatchesInliers[i])
                N++;

        // We recover 8 motion hypotheses using the method of Faugeras et al.
        // Motion and structure from motion in a piecewise planar environment.
        // International Journal of Pattern Recognition and Artificial Intelligence, 1988

        // 这个地方对应的是根据单应矩阵恢复R、t的部分
        cv::Mat invK = K.inv();
        cv::Mat A = invK * H21 * K;

        // SVD分解
        cv::Mat U, w, Vt, V;
        cv::SVD::compute(A, w, U, Vt, cv::SVD::FULL_UV);
        V = Vt.t();

        float s = cv::determinant(U) * cv::determinant(Vt);

        float d1 = w.at<float>(0);
        float d2 = w.at<float>(1);
        float d3 = w.at<float>(2);

        if (d1 / d2 < 1.00001 || d2 / d3 < 1.00001)
        {
            return false;
        }

        vector<cv::Mat> vR, vt, vn;
        vR.reserve(8);
        vt.reserve(8);
        vn.reserve(8);

        // n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
        // 可能包含有多种情况
        float aux1 = sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
        float aux3 = sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
        float x1[] = {aux1, aux1, -aux1, -aux1};
        float x3[] = {aux3, -aux3, aux3, -aux3};

        // case d'=d2
        float aux_stheta = sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);

        float ctheta = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
        float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

        // 分别计算和构造4中情况，分别存放在VR和vt里
        for (int i = 0; i < 4; i++)
        {
            cv::Mat Rp = cv::Mat::eye(3, 3, CV_32F);
            Rp.at<float>(0, 0) = ctheta;
            Rp.at<float>(0, 2) = -stheta[i];
            Rp.at<float>(2, 0) = stheta[i];
            Rp.at<float>(2, 2) = ctheta;

            cv::Mat R = s * U * Rp * Vt;
            vR.push_back(R); // 保存R

            cv::Mat tp(3, 1, CV_32F);
            tp.at<float>(0) = x1[i];
            tp.at<float>(1) = 0;
            tp.at<float>(2) = -x3[i];
            tp *= d1 - d3;

            cv::Mat t = U * tp;
            vt.push_back(t / cv::norm(t)); // 保存t

            cv::Mat np(3, 1, CV_32F);
            np.at<float>(0) = x1[i];
            np.at<float>(1) = 0;
            np.at<float>(2) = x3[i];

            cv::Mat n = V * np;
            if (n.at<float>(2) < 0)
                n = -n;
            vn.push_back(n);
        }

        // case d'=-d2
        float aux_sphi = sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);

        float cphi = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
        float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

        for (int i = 0; i < 4; i++)
        {
            cv::Mat Rp = cv::Mat::eye(3, 3, CV_32F);
            Rp.at<float>(0, 0) = cphi;
            Rp.at<float>(0, 2) = sphi[i];
            Rp.at<float>(1, 1) = -1;
            Rp.at<float>(2, 0) = sphi[i];
            Rp.at<float>(2, 2) = -cphi;

            cv::Mat R = s * U * Rp * Vt;
            vR.push_back(R);

            cv::Mat tp(3, 1, CV_32F);
            tp.at<float>(0) = x1[i];
            tp.at<float>(1) = 0;
            tp.at<float>(2) = x3[i];
            tp *= d1 + d3;

            cv::Mat t = U * tp;
            vt.push_back(t / cv::norm(t));

            cv::Mat np(3, 1, CV_32F);
            np.at<float>(0) = x1[i];
            np.at<float>(1) = 0;
            np.at<float>(2) = x3[i];

            cv::Mat n = V * np;
            if (n.at<float>(2) < 0)
                n = -n;
            vn.push_back(n);
        }

        int bestGood = 0;
        int secondBestGood = 0;
        int bestSolutionIdx = -1;
        float bestParallax = -1;
        vector<cv::Point3f> bestP3D;
        vector<bool> bestTriangulated;

        // 利用三角化的方法筛选可能解
        // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
        // We reconstruct all hypotheses and check in terms of triangulated points and parallax
        for (size_t i = 0; i < 8; i++)
        {
            float parallaxi;
            vector<cv::Point3f> vP3Di;
            vector<bool> vbTriangulatedi;
            // 检验R、t是否正确就是在这个函数里进行的，传入、传出的参数比较多
            int nGood = CheckRT(
                vR[i], vt[i],       // 传入参数，待验证的R、t
                mvKeys1, mvKeys2,   // 传入参数，两帧上提取的特征点
                mvMatches12,        // 传入参数，特征点间的匹配关系
                vbMatchesInliers,   // 传入参数，满足单应矩阵H约束的点对标记
                K,                  // 传出参数，相机内参矩阵
                vP3Di,              // 传出参数，成功三角化的三维特征点坐标
                4.0 * mSigma2,      // 传出参数，三角化检验师，重投影误差阈值
                vbTriangulatedi,    // 传出参数，成功三角化的点标记
                parallaxi);         // 传出参数，两帧间的视差（角度）

            // 分别记录最多正确三角化点的个数和第二多的个数
            if (nGood > bestGood)
            {
                secondBestGood = bestGood;
                bestGood = nGood;
                bestSolutionIdx = i;
                bestParallax = parallaxi;
                bestP3D = vP3Di;
                bestTriangulated = vbTriangulatedi;
            }
            else if (nGood > secondBestGood)
            {
                secondBestGood = nGood;
            }
        }

        // 必须同时满足以下几个条件，才认为R、t的分解是对的，并且返回对应状态下的正确三角化的特征点
        // 1.第二多的正确三角化点数量小于最多三角化点数量的0.75倍
        // 2.最好的帧间视差大于最小阈值
        // 3.最多的正确三角化点的个数大于最小个数阈值
        // 4.最多的正确三角化点的个数大于0.9倍的总数
        if (secondBestGood < 0.75 * bestGood && bestParallax >= minParallax && bestGood > minTriangulated && bestGood > 0.9 * N)
        {
            vR[bestSolutionIdx].copyTo(R21);
            vt[bestSolutionIdx].copyTo(t21);
            vP3D = bestP3D;
            vbTriangulated = bestTriangulated;

            return true;
        }

        return false;
    }

    void Initializer::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
    {
        // 核心算法，需要理解一下
        cv::Mat A(4, 4, CV_32F);

        // 构造系数矩阵
        A.row(0) = kp1.pt.x * P1.row(2) - P1.row(0);
        A.row(1) = kp1.pt.y * P1.row(2) - P1.row(1);
        A.row(2) = kp2.pt.x * P2.row(2) - P2.row(0);
        A.row(3) = kp2.pt.y * P2.row(2) - P2.row(1);

        cv::Mat u, w, vt;
        // 三角化算深度也可以看作是一个优化问题，所以构造稀疏矩阵，用SVD分解求解
        cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
        x3D = vt.row(3).t();
        x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
    }

    void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
    {
        // 整个标准化的步骤是将特征点坐标取出均值后再除以绝对矩
        // 归一化后的坐标均值为0，一阶绝对矩为1
        // 绝对矩的计算公式是mean_x_dev = sum(xi - x_mean)/N
        float meanX = 0;
        float meanY = 0;
        const int N = vKeys.size();

        vNormalizedPoints.resize(N);

        // 将输出的vector设置成和输入等长
        for (int i = 0; i < N; i++)
        {
            meanX += vKeys[i].pt.x;
            meanY += vKeys[i].pt.y;
        }

        // 计算x、y的均值
        meanX = meanX / N;
        meanY = meanY / N;

        float meanDevX = 0;
        float meanDevY = 0;

        for (int i = 0; i < N; i++)
        {
            // 将每一个x、y减去均值，并顺便求一下绝对矩
            vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
            vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

            // 注意这里计算的是绝对矩，不是方差
            meanDevX += fabs(vNormalizedPoints[i].x);
            meanDevY += fabs(vNormalizedPoints[i].y);
        }

        meanDevX = meanDevX / N;
        meanDevY = meanDevY / N;

        // 尺度缩放因子，为了使标准化后的坐标绝对矩为1，所以取绝对矩的倒数
        float sX = 1.0 / meanDevX;
        float sY = 1.0 / meanDevY;

        for (int i = 0; i < N; i++)
        {
            // 对每一个点再乘以尺度缩放因子
            // 标准化后的点坐标x = (u - mean_u) * sX, y = (v - mena_v) * sY
            vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
            vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
        }

        // 构造标准化变换矩阵T，注意它代表的变换方向，原始坐标乘以变换矩阵得到归一化坐标
        // 标准化后的坐标（齐次） = 标准化矩阵 * 原始坐标（齐次）
        // 原始坐标（齐次） = 标准化矩阵的逆 * 标准化后的坐标（齐次）
        T = cv::Mat::eye(3, 3, CV_32F);
        T.at<float>(0, 0) = sX;
        T.at<float>(1, 1) = sY;
        T.at<float>(0, 2) = -meanX * sX;
        T.at<float>(1, 2) = -meanY * sY;
    }

    int Initializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                             const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                             const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
    {
        // Calibration parameters   提取相机内参
        const float fx = K.at<float>(0, 0);
        const float fy = K.at<float>(1, 1);
        const float cx = K.at<float>(0, 2);
        const float cy = K.at<float>(1, 2);

        vbGood = vector<bool>(vKeys1.size(), false);
        vP3D.resize(vKeys1.size()); // vP3D的大小和vKeys1相同

        vector<float> vCosParallax;
        vCosParallax.reserve(vKeys1.size());

        // Camera 1 Projection Matrix K[I|0]    构造投影矩阵，将像素坐标转到三维相机坐标
        cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
        // 将K的内容拷贝到P1的对应部分
        K.copyTo(P1.rowRange(0, 3).colRange(0, 3));

        // 一个3*1，元素全为0的向量。指的是第一帧相机坐标系原点在第一帧相机坐标系下的坐标
        cv::Mat O1 = cv::Mat::zeros(3, 1, CV_32F);

        // Camera 2 Projection Matrix K[R|t]
        cv::Mat P2(3, 4, CV_32F);
        R.copyTo(P2.rowRange(0, 3).colRange(0, 3));
        t.copyTo(P2.rowRange(0, 3).col(3));
        P2 = K * P2;

        // R的转置（因为R是旋转矩阵，正交，所以转置等于逆）乘以t的结果再取负号
        // 指的是第二帧相机坐标系的远点在第一帧相机坐标系下的坐标
        cv::Mat O2 = -R.t() * t;

        int nGood = 0;

        // 对于匹配到的点对，依次进行三角化求深度
        for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
        {
            // 只三角化满足单应矩阵H的内点匹配对，对于其它的直接跳过
            if (!vbMatchesInliers[i])
                continue;

            // 还是之前说的，vMatches12里存放的是匹配点对分别在vKeys1和vKeys2中的索引，所以需要以这种方式获得实际点
            const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
            const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
            cv::Mat p3dC1;  // 在Camera1坐标系下的3d点坐标

            // 核心计算深度的函数
            Triangulate(kp1, kp2, P1, P2, p3dC1);

            // 如果求解出来的三维坐标中，有任何一维为无穷，认为是错的，三角化失败，将对应项赋为false
            if (!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
            {
                vbGood[vMatches12[i].first] = false;
                continue;
            }

            // Check parallax
            // 得到的是一个从第一帧相机坐标系原点指向p3dC1点的向量
            cv::Mat normal1 = p3dC1 - O1;
            // 计算向量的L2番薯（默认），可以理解为是该点到第一帧相机坐标系原点的距离或者说模长
            // 矩阵的L2范数为矩阵的各个元素平方之和再开平方，又被称为F范数（Frobenius Norm），欧式距离就是一种L2范数
            float dist1 = cv::norm(normal1);

            // 得到的是一个从第二帧相机坐标系原点指向p3dC1的向量
            cv::Mat normal2 = p3dC1 - O2;
            // 求解这个向量的L2范数（模长）
            float dist2 = cv::norm(normal2);

            // 求解视差
            // 其实这个地方用到的是向量点积公式：a·b = |a| * |b| * cos(theta)
            // 将这个公式变形一下，可以得到cos(theta) = a·b / (|a| * |b|)
            // 点乘的几何意义是可以用来表征或计算两个向量之间的夹角，以及在b向量在a向量方向上的投影
            // 所以这里判断视差其实是通过判断两个向量的夹角的cos值进行的
            // 具体来说上面首先分别构造了从第一帧相机坐标系原点指向特征点以及从第二帧相机坐标系原点指向特征点这样的两个向量
            // 然后运用向量点积公式的变形，就可以很容易求得这两个向量之间的夹角余弦
            // 最后通过对余弦大小的判别进而判断视差的大小
            float cosParallax = normal1.dot(normal2) / (dist1 * dist2);

            // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            // 结合上面提到的，这里判断条件之一是如果cos的值小于0.99998，就认为不靠谱了
            // 可以很容易根据反三角函数计算出来acos(0.99998)=0.006324
            // 0.006324*180/pi = 0.362°
            // 换句话说，如果两帧之间的视差角度小于0.361度就认为这样估算出来的三维点是不可靠的
            // 而且根据这个指标，也可以由几何关系推算出基线与深度的倍数关系，可以算出来大约是1:156.25
            // ? 1/acos(0.99998)=158.11
            // 此外如果按照40倍基线长度计算的话，可以大致算出来视差应该为1.4324度左右
            // 另一个判断条件是z坐标小于等于0（深度为负），认为是错的
            if (p3dC1.at<float>(2) <= 0 && cosParallax < 0.99998)
                continue;

            // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            cv::Mat p3dC2 = R * p3dC1 + t;

            // 和上一个评价指标一样，如果不满足就认为是错的
            if (p3dC2.at<float>(2) <= 0 && cosParallax < 0.99998)
                continue;

            // Check reprojection error in first image
            float im1x, im1y;
            // 由于是齐次坐标，所以需要对第三维归一化，先求出归一化因子
            float invZ1 = 1.0 / p3dC1.at<float>(2);
            // 结合归一化因子和相机内参，求得在图像平面上的像素坐标
            im1x = fx * p3dC1.at<float>(0) * invZ1 + cx;
            im1y = fy * p3dC1.at<float>(1) * invZ1 + cy;

            // 误差为dx平方+dy平方
            float squareError1 = (im1x - kp1.pt.x) * (im1x - kp1.pt.x) + (im1y - kp1.pt.y) * (im1y - kp1.pt.y);

            // 如果误差大于给定的阈值th2就认为是错误的
            if (squareError1 > th2)
                continue;

            // Check reprojection error in second image 再次检查一遍
            float im2x, im2y;
            float invZ2 = 1.0 / p3dC2.at<float>(2);
            im2x = fx * p3dC2.at<float>(0) * invZ2 + cx;
            im2y = fy * p3dC2.at<float>(1) * invZ2 + cy;

            // 同理计算投影误差
            float squareError2 = (im2x - kp2.pt.x) * (im2x - kp2.pt.x) + (im2y - kp2.pt.y) * (im2y - kp2.pt.y);
            // 如果误差大于阈值直接丢弃
            if (squareError2 > th2)
                continue;

            // 如果点对通过了以上的检验，就把它对应的视差存储起来
            vCosParallax.push_back(cosParallax);
            // 注意这里三维深度点赋值方法
            // vMatches12里存储的是匹配点对在各自图像特征点vector中的索引
            // 所以这里直接获取到对应特征点的索引
            // 由于vP3D和vKeys1是一样的，所以索引通用，直接赋值
            vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0), p3dC1.at<float>(1), p3dC1.at<float>(2));
            nGood++;    // 好的三角点数加1

            // 同时，再设置对应标签为true
            if (cosParallax < 0.99998)
                vbGood[vMatches12[i].first] = true;
        }
        // 这样循环一遍后，就可以得到当前R、t条件下的三角化点以及好的三角化点的个数

        // 如果说好的三角点的数量大于1，就进行下面步骤
        if (nGood > 0)
        {
            // 在上面循环中，把每一个成功三角化了的点对应的视差放到了vCosParallax里
            // 这里对这些视差从小到大排个序
            sort(vCosParallax.begin(), vCosParallax.end());

            // 如果可靠点的个数小于50，就选最大的索引；如果大于50；都选50
            size_t idx = min(50, int(vCosParallax.size() - 1));
            // 由于保存的是cos，所以用反三角函数求解出角度，再将其转化成度
            parallax = acos(vCosParallax[idx]) * 180 / CV_PI;
        }
        // 否则的话视差设为0度
        else
            parallax = 0;

        return nGood;
    }

    void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
    {
        cv::Mat u, w, vt;
        cv::SVD::compute(E, w, u, vt);

        u.col(2).copyTo(t);
        t = t / cv::norm(t);

        cv::Mat W(3, 3, CV_32F, cv::Scalar(0));
        W.at<float>(0, 1) = -1;
        W.at<float>(1, 0) = 1;
        W.at<float>(2, 2) = 1;

        R1 = u * W * vt;
        if (cv::determinant(R1) < 0)
            R1 = -R1;

        R2 = u * W.t() * vt;
        if (cv::determinant(R2) < 0)
            R2 = -R2;
    }

} // namespace ORB_SLAM

/**
 * This file is part of ORB-SLAM2.
 * This file is based on the file orb.cpp from the OpenCV library (see BSD license below).
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
/**
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>

#include "ORBextractor.h"

using namespace cv;
using namespace std;

namespace ORB_SLAM2
{
    // 预先定义好的一些参数常量
    const int PATCH_SIZE = 31;
    const int HALF_PATCH_SIZE = 15;
    const int EDGE_THRESHOLD = 19; // 图像扩边的时候用到

    static float IC_Angle(const Mat &image, Point2f pt, const vector<int> &u_max)
    {
        int m_01 = 0, m_10 = 0; // 它们对应公式中的常量

        // 先对传入的特征点坐标四舍五入取整，基于这个坐标再去获取一个指向该像素的指针，注意不是对应的灰度值
        // 把image前面的取地址符&去掉才是灰度值，这里center是一个uchar类型的指针，所以后面才能对它进行索引取值
        const uchar *center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));

        // Treat the center line differently, v=0
        for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
            m_10 += u * center[u];

        // Go line by line in the circuI853lar patch
        int step = (int)image.step1();
        for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
        {
            // Proceed over the two lines
            int v_sum = 0;
            int d = u_max[v];
            for (int u = -d; u <= d; ++u)
            {
                int val_plus = center[u + v * step], val_minus = center[u - v * step];
                v_sum += (val_plus - val_minus);
                m_10 += u * (val_plus + val_minus);
            }
            m_01 += v * v_sum;
        }

        return fastAtan2((float)m_01, (float)m_10);
    }

    const float factorPI = (float)(CV_PI / 180.f); // 角度转成弧度制的系数
    static void computeOrbDescriptor(const KeyPoint &kpt,
                                     const Mat &img, const Point *pattern,
                                     uchar *desc)
    // 传入的参数中带const的都是不会修改，只读的
    // 另外这里看起来传入的是一个Point指针类型的pattern，但其实是一个数组，可以根据索引获取其它元素
    {
        // 在特征提取后，会在computerOrientation函数中计算特征点的角度
        float angle = (float)kpt.angle * factorPI;          // 将角度转换成弧度
        float a = (float)cos(angle), b = (float)sin(angle); // 相关变量计算

        // 和之前一样，这里获取的是一个指向特征点的uchar类型的指针，并非是特征点所对应像素的灰度值，需要注意
        // 如果img前面的取地址符&去掉，获取的就是该位置对应的灰度值了
        // 换句话说是可以对center进行索引操作获取其它元素的值的，这在下面定义的宏中就有体现
        const uchar *center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
        // 这里的step是Mat的属性之一，表示一个数据所占字节长度，方便后面迭代时候跳过指定步长
        // 例如一个单通道灰度图像一个像素值在我这里字节长度是700，而RGB图像一个像素的长度是2100
        const int step = (int)img.step;

// 则合理定义了一个宏，用于后续计算的方便
// 这里的center就是上面获取的uchar的指针，而pattern就是传入的参数
// 这里面一长串其实都是在计算索引，而对center取索引获取到的就是该索引（位置）所对应的灰度值
#define GET_VALUE(idx)                                               \
    center[cvRound(pattern[idx].x * b + pattern[idx].y * a) * step + \
           cvRound(pattern[idx].x * a - pattern[idx].y * b)]

        // 前面定义的描述子有32列，对应这里的32次循环
        // 注意这里的pattern指向的地址随着迭代也在变化，每次增加16
        // 这16其实就是对应一次迭代读取pattern的步长，在一次迭代中，一共会获取16个元素，所以下一次迭代从16开始
        // 另外，根据这个算法，可以计算出一共要有32*16个点用于迭代，而在代码一开始的pattern中就是512个点，在类的构造函数初始化pattern的时候也有体现
        for (int i = 0; i < 32; ++i, pattern += 16)
        {
            int t0, t1, val;    // 临时变量
            // 获取0，1索引对应的像素的灰度值，然后进行比较，比较结果放到val里
            t0 = GET_VALUE(0);
            t1 = GET_VALUE(1);
            val = t0 < t1;      // true为1，false为0
            t0 = GET_VALUE(2);  // 和上面一样
            t1 = GET_VALUE(3);
            // |表示两个相应的二进制位中只要有一个为1，该位的结果值为1，否则为0
            // 而这里又多了一个等号，就类似于+=这种操作，表示按位或赋值
            // 将val于t0 < t1进行按位或运算，并将结果再赋给val
            // << 用来将一个数的各二进制位全部左移N位，高位舍弃，低位补0
            // 另外还需要主义的是这里的运算顺序
            // 这里认为添加了括号，所以先运算括号里面的内容，然后运算 <<，最后运算后的结果再按位或赋值
            // 这里之所以要左移1位是因为上面第一位已经有了内容，如果不左移的话，数据就被覆盖了
            val |= (t0 < t1) << 1;
            t0 = GET_VALUE(4);
            t1 = GET_VALUE(5);
            val |= (t0 < t1) << 2;
            t0 = GET_VALUE(6);
            t1 = GET_VALUE(7);
            val |= (t0 < t1) << 3;
            t0 = GET_VALUE(8);
            t1 = GET_VALUE(9);
            val |= (t0 < t1) << 4;
            t0 = GET_VALUE(10);
            t1 = GET_VALUE(11);
            val |= (t0 < t1) << 5;
            t0 = GET_VALUE(12);
            t1 = GET_VALUE(13);
            val |= (t0 < t1) << 6;
            t0 = GET_VALUE(14);
            t1 = GET_VALUE(15);
            val |= (t0 < t1) << 7;

            // 每次比较都有一个0或1的结果，上面比较了8次，因此到这里val是8位的0、1二进制串
            // 而且根据desc数据类型的定义，是8U，最大值为255，所以将8位的二进制串转成char类型不会有任何数据丢失问题
            // 另外前面说了，desc可以看成是有32个元素的数组，因此直接基于索引就可以进行赋值
            // 将二进制串转换成uchar类型的数值后就可以直接赋值给desc[1]，就完成了一个描述子的一个数的计算
            // 迭代32次就可以把整个描述子计算出来
            desc[i] = (uchar)val;
        }

// 结束定义的宏
#undef GET_VALUE
    }

    // 对应论文中提供好的pattern，直接用即可
    // 注意一下存放的数据结构，一共1024个数，对应512个点
    // bit_pattern_31_本身是一个长度位1024的一维数组
    // 但经过构造函数中的指针类型转换，就使得pattern对应的是一个512长度的Point类型的数组了
    // 最后将这个长度位512的Point类型的数组拷贝给vector成员变量pattern
    static int bit_pattern_31_[256 * 4] =
        {
            8, -3, 9, 5 /*mean (0), correlation (0)*/,
            4, 2, 7, -12 /*mean (1.12461e-05), correlation (0.0437584)*/,
            -11, 9, -8, 2 /*mean (3.37382e-05), correlation (0.0617409)*/,
            7, -12, 12, -13 /*mean (5.62303e-05), correlation (0.0636977)*/,
            2, -13, 2, 12 /*mean (0.000134953), correlation (0.085099)*/,
            1, -7, 1, 6 /*mean (0.000528565), correlation (0.0857175)*/,
            -2, -10, -2, -4 /*mean (0.0188821), correlation (0.0985774)*/,
            -13, -13, -11, -8 /*mean (0.0363135), correlation (0.0899616)*/,
            -13, -3, -12, -9 /*mean (0.121806), correlation (0.099849)*/,
            10, 4, 11, 9 /*mean (0.122065), correlation (0.093285)*/,
            -13, -8, -8, -9 /*mean (0.162787), correlation (0.0942748)*/,
            -11, 7, -9, 12 /*mean (0.21561), correlation (0.0974438)*/,
            7, 7, 12, 6 /*mean (0.160583), correlation (0.130064)*/,
            -4, -5, -3, 0 /*mean (0.228171), correlation (0.132998)*/,
            -13, 2, -12, -3 /*mean (0.00997526), correlation (0.145926)*/,
            -9, 0, -7, 5 /*mean (0.198234), correlation (0.143636)*/,
            12, -6, 12, -1 /*mean (0.0676226), correlation (0.16689)*/,
            -3, 6, -2, 12 /*mean (0.166847), correlation (0.171682)*/,
            -6, -13, -4, -8 /*mean (0.101215), correlation (0.179716)*/,
            11, -13, 12, -8 /*mean (0.200641), correlation (0.192279)*/,
            4, 7, 5, 1 /*mean (0.205106), correlation (0.186848)*/,
            5, -3, 10, -3 /*mean (0.234908), correlation (0.192319)*/,
            3, -7, 6, 12 /*mean (0.0709964), correlation (0.210872)*/,
            -8, -7, -6, -2 /*mean (0.0939834), correlation (0.212589)*/,
            -2, 11, -1, -10 /*mean (0.127778), correlation (0.20866)*/,
            -13, 12, -8, 10 /*mean (0.14783), correlation (0.206356)*/,
            -7, 3, -5, -3 /*mean (0.182141), correlation (0.198942)*/,
            -4, 2, -3, 7 /*mean (0.188237), correlation (0.21384)*/,
            -10, -12, -6, 11 /*mean (0.14865), correlation (0.23571)*/,
            5, -12, 6, -7 /*mean (0.222312), correlation (0.23324)*/,
            5, -6, 7, -1 /*mean (0.229082), correlation (0.23389)*/,
            1, 0, 4, -5 /*mean (0.241577), correlation (0.215286)*/,
            9, 11, 11, -13 /*mean (0.00338507), correlation (0.251373)*/,
            4, 7, 4, 12 /*mean (0.131005), correlation (0.257622)*/,
            2, -1, 4, 4 /*mean (0.152755), correlation (0.255205)*/,
            -4, -12, -2, 7 /*mean (0.182771), correlation (0.244867)*/,
            -8, -5, -7, -10 /*mean (0.186898), correlation (0.23901)*/,
            4, 11, 9, 12 /*mean (0.226226), correlation (0.258255)*/,
            0, -8, 1, -13 /*mean (0.0897886), correlation (0.274827)*/,
            -13, -2, -8, 2 /*mean (0.148774), correlation (0.28065)*/,
            -3, -2, -2, 3 /*mean (0.153048), correlation (0.283063)*/,
            -6, 9, -4, -9 /*mean (0.169523), correlation (0.278248)*/,
            8, 12, 10, 7 /*mean (0.225337), correlation (0.282851)*/,
            0, 9, 1, 3 /*mean (0.226687), correlation (0.278734)*/,
            7, -5, 11, -10 /*mean (0.00693882), correlation (0.305161)*/,
            -13, -6, -11, 0 /*mean (0.0227283), correlation (0.300181)*/,
            10, 7, 12, 1 /*mean (0.125517), correlation (0.31089)*/,
            -6, -3, -6, 12 /*mean (0.131748), correlation (0.312779)*/,
            10, -9, 12, -4 /*mean (0.144827), correlation (0.292797)*/,
            -13, 8, -8, -12 /*mean (0.149202), correlation (0.308918)*/,
            -13, 0, -8, -4 /*mean (0.160909), correlation (0.310013)*/,
            3, 3, 7, 8 /*mean (0.177755), correlation (0.309394)*/,
            5, 7, 10, -7 /*mean (0.212337), correlation (0.310315)*/,
            -1, 7, 1, -12 /*mean (0.214429), correlation (0.311933)*/,
            3, -10, 5, 6 /*mean (0.235807), correlation (0.313104)*/,
            2, -4, 3, -10 /*mean (0.00494827), correlation (0.344948)*/,
            -13, 0, -13, 5 /*mean (0.0549145), correlation (0.344675)*/,
            -13, -7, -12, 12 /*mean (0.103385), correlation (0.342715)*/,
            -13, 3, -11, 8 /*mean (0.134222), correlation (0.322922)*/,
            -7, 12, -4, 7 /*mean (0.153284), correlation (0.337061)*/,
            6, -10, 12, 8 /*mean (0.154881), correlation (0.329257)*/,
            -9, -1, -7, -6 /*mean (0.200967), correlation (0.33312)*/,
            -2, -5, 0, 12 /*mean (0.201518), correlation (0.340635)*/,
            -12, 5, -7, 5 /*mean (0.207805), correlation (0.335631)*/,
            3, -10, 8, -13 /*mean (0.224438), correlation (0.34504)*/,
            -7, -7, -4, 5 /*mean (0.239361), correlation (0.338053)*/,
            -3, -2, -1, -7 /*mean (0.240744), correlation (0.344322)*/,
            2, 9, 5, -11 /*mean (0.242949), correlation (0.34145)*/,
            -11, -13, -5, -13 /*mean (0.244028), correlation (0.336861)*/,
            -1, 6, 0, -1 /*mean (0.247571), correlation (0.343684)*/,
            5, -3, 5, 2 /*mean (0.000697256), correlation (0.357265)*/,
            -4, -13, -4, 12 /*mean (0.00213675), correlation (0.373827)*/,
            -9, -6, -9, 6 /*mean (0.0126856), correlation (0.373938)*/,
            -12, -10, -8, -4 /*mean (0.0152497), correlation (0.364237)*/,
            10, 2, 12, -3 /*mean (0.0299933), correlation (0.345292)*/,
            7, 12, 12, 12 /*mean (0.0307242), correlation (0.366299)*/,
            -7, -13, -6, 5 /*mean (0.0534975), correlation (0.368357)*/,
            -4, 9, -3, 4 /*mean (0.099865), correlation (0.372276)*/,
            7, -1, 12, 2 /*mean (0.117083), correlation (0.364529)*/,
            -7, 6, -5, 1 /*mean (0.126125), correlation (0.369606)*/,
            -13, 11, -12, 5 /*mean (0.130364), correlation (0.358502)*/,
            -3, 7, -2, -6 /*mean (0.131691), correlation (0.375531)*/,
            7, -8, 12, -7 /*mean (0.160166), correlation (0.379508)*/,
            -13, -7, -11, -12 /*mean (0.167848), correlation (0.353343)*/,
            1, -3, 12, 12 /*mean (0.183378), correlation (0.371916)*/,
            2, -6, 3, 0 /*mean (0.228711), correlation (0.371761)*/,
            -4, 3, -2, -13 /*mean (0.247211), correlation (0.364063)*/,
            -1, -13, 1, 9 /*mean (0.249325), correlation (0.378139)*/,
            7, 1, 8, -6 /*mean (0.000652272), correlation (0.411682)*/,
            1, -1, 3, 12 /*mean (0.00248538), correlation (0.392988)*/,
            9, 1, 12, 6 /*mean (0.0206815), correlation (0.386106)*/,
            -1, -9, -1, 3 /*mean (0.0364485), correlation (0.410752)*/,
            -13, -13, -10, 5 /*mean (0.0376068), correlation (0.398374)*/,
            7, 7, 10, 12 /*mean (0.0424202), correlation (0.405663)*/,
            12, -5, 12, 9 /*mean (0.0942645), correlation (0.410422)*/,
            6, 3, 7, 11 /*mean (0.1074), correlation (0.413224)*/,
            5, -13, 6, 10 /*mean (0.109256), correlation (0.408646)*/,
            2, -12, 2, 3 /*mean (0.131691), correlation (0.416076)*/,
            3, 8, 4, -6 /*mean (0.165081), correlation (0.417569)*/,
            2, 6, 12, -13 /*mean (0.171874), correlation (0.408471)*/,
            9, -12, 10, 3 /*mean (0.175146), correlation (0.41296)*/,
            -8, 4, -7, 9 /*mean (0.183682), correlation (0.402956)*/,
            -11, 12, -4, -6 /*mean (0.184672), correlation (0.416125)*/,
            1, 12, 2, -8 /*mean (0.191487), correlation (0.386696)*/,
            6, -9, 7, -4 /*mean (0.192668), correlation (0.394771)*/,
            2, 3, 3, -2 /*mean (0.200157), correlation (0.408303)*/,
            6, 3, 11, 0 /*mean (0.204588), correlation (0.411762)*/,
            3, -3, 8, -8 /*mean (0.205904), correlation (0.416294)*/,
            7, 8, 9, 3 /*mean (0.213237), correlation (0.409306)*/,
            -11, -5, -6, -4 /*mean (0.243444), correlation (0.395069)*/,
            -10, 11, -5, 10 /*mean (0.247672), correlation (0.413392)*/,
            -5, -8, -3, 12 /*mean (0.24774), correlation (0.411416)*/,
            -10, 5, -9, 0 /*mean (0.00213675), correlation (0.454003)*/,
            8, -1, 12, -6 /*mean (0.0293635), correlation (0.455368)*/,
            4, -6, 6, -11 /*mean (0.0404971), correlation (0.457393)*/,
            -10, 12, -8, 7 /*mean (0.0481107), correlation (0.448364)*/,
            4, -2, 6, 7 /*mean (0.050641), correlation (0.455019)*/,
            -2, 0, -2, 12 /*mean (0.0525978), correlation (0.44338)*/,
            -5, -8, -5, 2 /*mean (0.0629667), correlation (0.457096)*/,
            7, -6, 10, 12 /*mean (0.0653846), correlation (0.445623)*/,
            -9, -13, -8, -8 /*mean (0.0858749), correlation (0.449789)*/,
            -5, -13, -5, -2 /*mean (0.122402), correlation (0.450201)*/,
            8, -8, 9, -13 /*mean (0.125416), correlation (0.453224)*/,
            -9, -11, -9, 0 /*mean (0.130128), correlation (0.458724)*/,
            1, -8, 1, -2 /*mean (0.132467), correlation (0.440133)*/,
            7, -4, 9, 1 /*mean (0.132692), correlation (0.454)*/,
            -2, 1, -1, -4 /*mean (0.135695), correlation (0.455739)*/,
            11, -6, 12, -11 /*mean (0.142904), correlation (0.446114)*/,
            -12, -9, -6, 4 /*mean (0.146165), correlation (0.451473)*/,
            3, 7, 7, 12 /*mean (0.147627), correlation (0.456643)*/,
            5, 5, 10, 8 /*mean (0.152901), correlation (0.455036)*/,
            0, -4, 2, 8 /*mean (0.167083), correlation (0.459315)*/,
            -9, 12, -5, -13 /*mean (0.173234), correlation (0.454706)*/,
            0, 7, 2, 12 /*mean (0.18312), correlation (0.433855)*/,
            -1, 2, 1, 7 /*mean (0.185504), correlation (0.443838)*/,
            5, 11, 7, -9 /*mean (0.185706), correlation (0.451123)*/,
            3, 5, 6, -8 /*mean (0.188968), correlation (0.455808)*/,
            -13, -4, -8, 9 /*mean (0.191667), correlation (0.459128)*/,
            -5, 9, -3, -3 /*mean (0.193196), correlation (0.458364)*/,
            -4, -7, -3, -12 /*mean (0.196536), correlation (0.455782)*/,
            6, 5, 8, 0 /*mean (0.1972), correlation (0.450481)*/,
            -7, 6, -6, 12 /*mean (0.199438), correlation (0.458156)*/,
            -13, 6, -5, -2 /*mean (0.211224), correlation (0.449548)*/,
            1, -10, 3, 10 /*mean (0.211718), correlation (0.440606)*/,
            4, 1, 8, -4 /*mean (0.213034), correlation (0.443177)*/,
            -2, -2, 2, -13 /*mean (0.234334), correlation (0.455304)*/,
            2, -12, 12, 12 /*mean (0.235684), correlation (0.443436)*/,
            -2, -13, 0, -6 /*mean (0.237674), correlation (0.452525)*/,
            4, 1, 9, 3 /*mean (0.23962), correlation (0.444824)*/,
            -6, -10, -3, -5 /*mean (0.248459), correlation (0.439621)*/,
            -3, -13, -1, 1 /*mean (0.249505), correlation (0.456666)*/,
            7, 5, 12, -11 /*mean (0.00119208), correlation (0.495466)*/,
            4, -2, 5, -7 /*mean (0.00372245), correlation (0.484214)*/,
            -13, 9, -9, -5 /*mean (0.00741116), correlation (0.499854)*/,
            7, 1, 8, 6 /*mean (0.0208952), correlation (0.499773)*/,
            7, -8, 7, 6 /*mean (0.0220085), correlation (0.501609)*/,
            -7, -4, -7, 1 /*mean (0.0233806), correlation (0.496568)*/,
            -8, 11, -7, -8 /*mean (0.0236505), correlation (0.489719)*/,
            -13, 6, -12, -8 /*mean (0.0268781), correlation (0.503487)*/,
            2, 4, 3, 9 /*mean (0.0323324), correlation (0.501938)*/,
            10, -5, 12, 3 /*mean (0.0399235), correlation (0.494029)*/,
            -6, -5, -6, 7 /*mean (0.0420153), correlation (0.486579)*/,
            8, -3, 9, -8 /*mean (0.0548021), correlation (0.484237)*/,
            2, -12, 2, 8 /*mean (0.0616622), correlation (0.496642)*/,
            -11, -2, -10, 3 /*mean (0.0627755), correlation (0.498563)*/,
            -12, -13, -7, -9 /*mean (0.0829622), correlation (0.495491)*/,
            -11, 0, -10, -5 /*mean (0.0843342), correlation (0.487146)*/,
            5, -3, 11, 8 /*mean (0.0929937), correlation (0.502315)*/,
            -2, -13, -1, 12 /*mean (0.113327), correlation (0.48941)*/,
            -1, -8, 0, 9 /*mean (0.132119), correlation (0.467268)*/,
            -13, -11, -12, -5 /*mean (0.136269), correlation (0.498771)*/,
            -10, -2, -10, 11 /*mean (0.142173), correlation (0.498714)*/,
            -3, 9, -2, -13 /*mean (0.144141), correlation (0.491973)*/,
            2, -3, 3, 2 /*mean (0.14892), correlation (0.500782)*/,
            -9, -13, -4, 0 /*mean (0.150371), correlation (0.498211)*/,
            -4, 6, -3, -10 /*mean (0.152159), correlation (0.495547)*/,
            -4, 12, -2, -7 /*mean (0.156152), correlation (0.496925)*/,
            -6, -11, -4, 9 /*mean (0.15749), correlation (0.499222)*/,
            6, -3, 6, 11 /*mean (0.159211), correlation (0.503821)*/,
            -13, 11, -5, 5 /*mean (0.162427), correlation (0.501907)*/,
            11, 11, 12, 6 /*mean (0.16652), correlation (0.497632)*/,
            7, -5, 12, -2 /*mean (0.169141), correlation (0.484474)*/,
            -1, 12, 0, 7 /*mean (0.169456), correlation (0.495339)*/,
            -4, -8, -3, -2 /*mean (0.171457), correlation (0.487251)*/,
            -7, 1, -6, 7 /*mean (0.175), correlation (0.500024)*/,
            -13, -12, -8, -13 /*mean (0.175866), correlation (0.497523)*/,
            -7, -2, -6, -8 /*mean (0.178273), correlation (0.501854)*/,
            -8, 5, -6, -9 /*mean (0.181107), correlation (0.494888)*/,
            -5, -1, -4, 5 /*mean (0.190227), correlation (0.482557)*/,
            -13, 7, -8, 10 /*mean (0.196739), correlation (0.496503)*/,
            1, 5, 5, -13 /*mean (0.19973), correlation (0.499759)*/,
            1, 0, 10, -13 /*mean (0.204465), correlation (0.49873)*/,
            9, 12, 10, -1 /*mean (0.209334), correlation (0.49063)*/,
            5, -8, 10, -9 /*mean (0.211134), correlation (0.503011)*/,
            -1, 11, 1, -13 /*mean (0.212), correlation (0.499414)*/,
            -9, -3, -6, 2 /*mean (0.212168), correlation (0.480739)*/,
            -1, -10, 1, 12 /*mean (0.212731), correlation (0.502523)*/,
            -13, 1, -8, -10 /*mean (0.21327), correlation (0.489786)*/,
            8, -11, 10, -6 /*mean (0.214159), correlation (0.488246)*/,
            2, -13, 3, -6 /*mean (0.216993), correlation (0.50287)*/,
            7, -13, 12, -9 /*mean (0.223639), correlation (0.470502)*/,
            -10, -10, -5, -7 /*mean (0.224089), correlation (0.500852)*/,
            -10, -8, -8, -13 /*mean (0.228666), correlation (0.502629)*/,
            4, -6, 8, 5 /*mean (0.22906), correlation (0.498305)*/,
            3, 12, 8, -13 /*mean (0.233378), correlation (0.503825)*/,
            -4, 2, -3, -3 /*mean (0.234323), correlation (0.476692)*/,
            5, -13, 10, -12 /*mean (0.236392), correlation (0.475462)*/,
            4, -13, 5, -1 /*mean (0.236842), correlation (0.504132)*/,
            -9, 9, -4, 3 /*mean (0.236977), correlation (0.497739)*/,
            0, 3, 3, -9 /*mean (0.24314), correlation (0.499398)*/,
            -12, 1, -6, 1 /*mean (0.243297), correlation (0.489447)*/,
            3, 2, 4, -8 /*mean (0.00155196), correlation (0.553496)*/,
            -10, -10, -10, 9 /*mean (0.00239541), correlation (0.54297)*/,
            8, -13, 12, 12 /*mean (0.0034413), correlation (0.544361)*/,
            -8, -12, -6, -5 /*mean (0.003565), correlation (0.551225)*/,
            2, 2, 3, 7 /*mean (0.00835583), correlation (0.55285)*/,
            10, 6, 11, -8 /*mean (0.00885065), correlation (0.540913)*/,
            6, 8, 8, -12 /*mean (0.0101552), correlation (0.551085)*/,
            -7, 10, -6, 5 /*mean (0.0102227), correlation (0.533635)*/,
            -3, -9, -3, 9 /*mean (0.0110211), correlation (0.543121)*/,
            -1, -13, -1, 5 /*mean (0.0113473), correlation (0.550173)*/,
            -3, -7, -3, 4 /*mean (0.0140913), correlation (0.554774)*/,
            -8, -2, -8, 3 /*mean (0.017049), correlation (0.55461)*/,
            4, 2, 12, 12 /*mean (0.01778), correlation (0.546921)*/,
            2, -5, 3, 11 /*mean (0.0224022), correlation (0.549667)*/,
            6, -9, 11, -13 /*mean (0.029161), correlation (0.546295)*/,
            3, -1, 7, 12 /*mean (0.0303081), correlation (0.548599)*/,
            11, -1, 12, 4 /*mean (0.0355151), correlation (0.523943)*/,
            -3, 0, -3, 6 /*mean (0.0417904), correlation (0.543395)*/,
            4, -11, 4, 12 /*mean (0.0487292), correlation (0.542818)*/,
            2, -4, 2, 1 /*mean (0.0575124), correlation (0.554888)*/,
            -10, -6, -8, 1 /*mean (0.0594242), correlation (0.544026)*/,
            -13, 7, -11, 1 /*mean (0.0597391), correlation (0.550524)*/,
            -13, 12, -11, -13 /*mean (0.0608974), correlation (0.55383)*/,
            6, 0, 11, -13 /*mean (0.065126), correlation (0.552006)*/,
            0, -1, 1, 4 /*mean (0.074224), correlation (0.546372)*/,
            -13, 3, -9, -2 /*mean (0.0808592), correlation (0.554875)*/,
            -9, 8, -6, -3 /*mean (0.0883378), correlation (0.551178)*/,
            -13, -6, -8, -2 /*mean (0.0901035), correlation (0.548446)*/,
            5, -9, 8, 10 /*mean (0.0949843), correlation (0.554694)*/,
            2, 7, 3, -9 /*mean (0.0994152), correlation (0.550979)*/,
            -1, -6, -1, -1 /*mean (0.10045), correlation (0.552714)*/,
            9, 5, 11, -2 /*mean (0.100686), correlation (0.552594)*/,
            11, -3, 12, -8 /*mean (0.101091), correlation (0.532394)*/,
            3, 0, 3, 5 /*mean (0.101147), correlation (0.525576)*/,
            -1, 4, 0, 10 /*mean (0.105263), correlation (0.531498)*/,
            3, -6, 4, 5 /*mean (0.110785), correlation (0.540491)*/,
            -13, 0, -10, 5 /*mean (0.112798), correlation (0.536582)*/,
            5, 8, 12, 11 /*mean (0.114181), correlation (0.555793)*/,
            8, 9, 9, -6 /*mean (0.117431), correlation (0.553763)*/,
            7, -4, 8, -12 /*mean (0.118522), correlation (0.553452)*/,
            -10, 4, -10, 9 /*mean (0.12094), correlation (0.554785)*/,
            7, 3, 12, 4 /*mean (0.122582), correlation (0.555825)*/,
            9, -7, 10, -2 /*mean (0.124978), correlation (0.549846)*/,
            7, 0, 12, -2 /*mean (0.127002), correlation (0.537452)*/,
            -1, -6, 0, -11 /*mean (0.127148), correlation (0.547401)*/
    };
    // 特征点的个数，比例金字塔中每层之间的比例因子，比例金字塔的层数，
    ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
                               int _iniThFAST, int _minThFAST) : nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
                                                                 iniThFAST(_iniThFAST), minThFAST(_minThFAST)
    {
        // 所以ORB参数再构造函数中就对应赋给了各个成员变量，所以后续就可以直接用了
        // mvScaleFactor和mvLevelSigma2是ORBextractor的成员变量vector，根据传入的层数设置了他的长度
        mvScaleFactor.resize(nlevels);
        mvLevelSigma2.resize(nlevels);
        // 它们的初值都是1
        mvScaleFactor[0] = 1.0f;
        mvLevelSigma2[0] = 1.0f;

        // 尺度按照ScaleFactor不断放大
        // 逆尺度按照ScaleFactor不断缩小

        // 初始化比例金字塔每一层的比例
        for (int i = 1; i < nlevels; i++)
        {
            mvScaleFactor[i] = mvScaleFactor[i - 1] * scaleFactor;  // 比例金字塔每一层的尺度因子
            mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i]; // 比例金字塔每一层尺度因子的平方
        }

        mvInvScaleFactor.resize(nlevels);
        mvInvLevelSigma2.resize(nlevels);
        for (int i = 0; i < nlevels; i++)
        {
            mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i]; // 比例金字塔每一层的逆尺度
            mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i]; // 比例金字塔每一层逆尺度的平方
        }

        mvImagePyramid.resize(nlevels); // 存储每一层金字塔图像的内容

        mnFeaturesPerLevel.resize(nlevels); // 存储每一层金字塔预设的特征点数量
        float factor = 1.0f / scaleFactor;
        float nDesiredFeaturesPerScale = nfeatures * (1 - factor) / (1 - (float)pow((double)factor, (double)nlevels));

        // 等比数列的求和公式
        // nfeatures = a1*(1-q^n)/(1-q)
        // a1 = nfeatures*(1-q)/(1-q^n)
        // a2 = nfeatures*(1-q)/(1-q^n)*q
        // ...
        // an = nfeatures*(1-q)/(1-q^n)*q^(n-1) = 0
        // (1-f)/(1-f^8)(1+f+f^2+f^3+f^4+f^5+f^6+f^7)
        //=(1-f)/(1-f^8)(1+f)(1+f^2)(1+f^4)
        //=(1-f)/(1-f^8)(1-f^8)/(1-f)
        //=1
        // nfeatures并不是每一层的特征点的数量，而是所有层特征点加一起的总数
        int sumFeatures = 0;
        for (int level = 0; level < nlevels - 1; level++)
        {
            mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
            sumFeatures += mnFeaturesPerLevel[level];
            nDesiredFeaturesPerScale *= factor;
        }

        // cvRound是一个四舍五入的函数。最后一层金字塔的特征点数=nfeatures-前几层金字塔特征点数量的和
        mnFeaturesPerLevel[nlevels - 1] = std::max(nfeatures - sumFeatures, 0);

        // 将1024长度的int型数组(bit_pattern_31_)变成了一个长度512的Point类型的一维数组(pattern0)
        // 程序会自动将两个数字组合再一起形成一个Point，例如
        // 8, -3, 9, 5 => (8,-3), (9,5)
        const int npoints = 512;
        const Point *pattern0 = (const Point *)bit_pattern_31_;
        // copy(待拷贝序列的首地址，待拷贝序列的最后一个元素的下一个地址，拷贝目标容器的首地址)
        std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

        // This is for orientation
        // pre-compute the end of a row in a circular patch
        // 这是用于方向。预先计算圆形补丁中的行尾
        umax.resize(HALF_PATCH_SIZE + 1);

        // cvFloor只会返回一个值给vmax，v, v0并没有赋初值
        int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
        int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
        const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
        for (v = 0; v <= vmax; ++v)
            umax[v] = cvRound(sqrt(hp2 - v * v));

        // Make sure we are symmetric
        // 确保我们是对称的
        for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
        {
            while (umax[v0] == umax[v0 + 1])
                ++v0;
            umax[v] = v0;
            ++v0;
        }
    }

    static void computeOrientation(const Mat &image, vector<KeyPoint> &keypoints, const vector<int> &umax)
    {
        // 其实核心是IC_Angle函数，这里是用了这个函数
        for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                                        keypointEnd = keypoints.end();
             keypoint != keypointEnd; ++keypoint)
        {
            // OpenCV的KeyPoint有个angle属性(float)，当就算好方向后就直接赋值给它
            // angle属性用角度表示，范围是[0,360)，顺时针
            keypoint->angle = IC_Angle(image, keypoint->pt, umax);
        }
    }

    void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
    {
        // (右下-左下)/2，向上取整，得到该节点的一半宽度
        // 同理再求得节点的一半高度
        // 至于static_cast可以简单理解为强制类型转换，这里就是将结果转换为float类型
        const int halfX = ceil(static_cast<float>(UR.x - UL.x) / 2);
        const int halfY = ceil(static_cast<float>(BR.y - UL.y) / 2);

        // Define boundaries of childs
        // 依次定义四个节点
        // 一堆节点边界相关的内容可以不用深究
        // 需要注意一下的是4个节点的顺序问题，是按行排列的
        // 1 2
        // 3 4
        n1.UL = UL;
        n1.UR = cv::Point2i(UL.x + halfX, UL.y);
        n1.BL = cv::Point2i(UL.x, UL.y + halfY);
        n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
        n1.vKeys.reserve(vKeys.size());

        // 这里意思就是说给新的子节点用于存放特征点的vector成员变量保留了父节点特征点数量的长度
        // 这里需要注意的是父节点的vKeys虽然没有显式传入，但因为是类的成员函数，所以可以直接调用
        // 另外，由于传入的四个节点是引用传递，因此在这里修改也就一改全改了，所以整个函数也没有显式的返回值
        n2.UL = n1.UR;
        n2.UR = UR;
        n2.BL = n1.BR;
        n2.BR = cv::Point2i(UR.x, UL.y + halfY);
        n2.vKeys.reserve(vKeys.size());

        n3.UL = n1.BL;
        n3.UR = n1.BR;
        n3.BL = BL;
        n3.BR = cv::Point2i(n1.BR.x, BL.y);
        n3.vKeys.reserve(vKeys.size());

        n4.UL = n3.UR;
        n4.UR = n2.BR;
        n4.BL = n3.BR;
        n4.BR = BR;
        n4.vKeys.reserve(vKeys.size());

        // Associate points to childs
        // 这一步也非常好理解，上面将父节点拆分成了4个子节点，还设置了子节点特征的vector的长度
        // 这里就是依次遍历父节点的特征点vector，根据特征点坐标将它们分配到不同节点
        for (size_t i = 0; i < vKeys.size(); i++)
        {
            const cv::KeyPoint &kp = vKeys[i];
            // 根据特征点坐标判断属于哪个子节点，这里先判断了x，再判断y
            if (kp.pt.x < n1.UR.x)
            {
                if (kp.pt.y < n1.BR.y)
                    n1.vKeys.push_back(kp);
                else
                    n3.vKeys.push_back(kp);
            }
            else if (kp.pt.y < n1.BR.y)
                n2.vKeys.push_back(kp);
            else
                n4.vKeys.push_back(kp);
        }

        // 各个子节点，如果只包含了一个特征点，bNoMore成员变量就设为true
        // 换句话说就是当前节点只包含了一个特征点了，就不要再往下继续分了
        // bNoMore的作用就是指明当前节点是否还可以继续分割
        if (n1.vKeys.size() == 1)
            n1.bNoMore = true;
        if (n2.vKeys.size() == 1)
            n2.bNoMore = true;
        if (n3.vKeys.size() == 1)
            n3.bNoMore = true;
        if (n4.vKeys.size() == 1)
            n4.bNoMore = true;
    }

    vector<cv::KeyPoint> ORBextractor::DistributeOctTree(const vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                                         const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
    {
        // Compute how many initial nodes
        // 前面说过了，static_cast理解为强制类型转换就好
        // 求解X方向上初始节点的个数，至于为什么用dx/dy再四舍五入取整这样比较奇怪的方式计算X方向初始节点个数暂时不知道
        const int nIni = round(static_cast<float>(maxX - minX) / (maxY - minY));
        // 根据上面获得的X方向上的节点个数求解初始节点的水平宽度
        const float hX = static_cast<float>(maxX - minX) / nIni;

        // 新建一个ExtractorNode类型的list用于存放节点
        // 这个list后面会频繁用到
        list<ExtractorNode> lNodes;

        // 新建一个Extractor指针类型的vector
        // 它里面存放的是每次循环向lNodes添加元素后其最后一个元素的地址
        vector<ExtractorNode *> vpIniNodes;
        vpIniNodes.resize(nIni);    // 长度就设置为刚刚计算出来的个数

        // 依次循环，构造节点放入vector中
        // 执行完这个循环其实只是将图像进行了X方向上的节点划分
        // 因为在循环中所有节点的四角点的Y坐标值都是对应相同的
        for (int i = 0; i < nIni; i++)
        {
            // 每次循环都新建一个临时变量ni，四角点坐标的计算比较简单，一看就懂
            ExtractorNode ni;
            // 上面两个坐标，所以y方向都为0
            ni.UL = cv::Point2i(hX * static_cast<float>(i), 0);
            ni.UR = cv::Point2i(hX * static_cast<float>(i + 1), 0);
            // 下面两个坐标，所以y方向都为dy，也就是maxY-minY
            ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
            ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
            // 设置节点的特征点vector长度为传入的vector的长度
            // 不要忘记了这个传入的vToDistributeKeys是哪里来的
            // 它里面是金字塔每一层提取的未经处理的原始特征点
            ni.vKeys.reserve(vToDistributeKeys.size());

            lNodes.push_back(ni);   // 将节点放入lNodes中
            // back()返回lNodes的最后一个元素的可读写的引用，换句话说你可以直接利用lNodes.back()修改最后一个元素的值
            // 但这里需要注意的是vpIniNodes是ExtractorNode指针型的vector，因此要在前面加个取地址符才可以
            vpIniNodes[i] = &lNodes.back();
        }

        // Associate points to childs
        // 遍历所有特征点，按照X方向上的位置安放到对应的节点中去
        for (size_t i = 0; i < vToDistributeKeys.size(); i++)
        {
            // 获取特征点
            const cv::KeyPoint &kp = vToDistributeKeys[i];
            // 根据特征点x坐标和节点宽度计算得到其所对应的节点索引
            // 再根据节点索引获得对应节点，利用 -> 操作符获取节点对象内的public的vector成员变量
            // 最后将该特征点push到这个vector中
            vpIniNodes[kp.pt.x / hX]->vKeys.push_back(kp);
        }

        // 构造一个迭代器，指向lNodes的第一个元素 ，后面就不新建一直用这个了
        list<ExtractorNode>::iterator lit = lNodes.begin();

        // 循环遍历节点，对于只含一个特征点和不含特征点的节点单独进行处理
        while (lit != lNodes.end())
        {
            // 如果当前节点只含一个特征点，那么就把当前节点的bNoMore设置为true
            // 也就是说指定当前节点不可以再继续分割了
            if (lit->vKeys.size() == 1)
            {
                lit->bNoMore = true;
                lit++;  // 迭代器迭代到下一个
            }
            // 如果说当前节点不含特征点，那么就直接把当前节点从lNodes中抹去
            else if (lit->vKeys.empty())
                // erase()函数的用法需要注意一下
                // 它的输入参数是指向需要抹去元素的迭代器
                // 而它相比于remove()函数无返回值，它是由返回值的
                // 它的返回值就是指向下一个元素的迭代器
                // 因此，在这个分支里就不再需要lit++了
                lit = lNodes.erase(lit);
            // 如果当前节点既不为空也不仅包含一个特征点，那么就什么都不做，直接跳过
            else
                lit++;
        }

        bool bFinish = false;   // 一个flag变量，用于指示分割是否完成，是否停止迭代

        int iteration = 0;  // 迭代次数

        // 一个vector用于存放尺寸与指向该节点的指针，长度设置为当前节点总数的4倍
        vector<pair<int, ExtractorNode *>> vSizeAndPointerToNode;
        vSizeAndPointerToNode.reserve(lNodes.size() * 4);

        // 开始迭代
        while (!bFinish)
        {
            iteration++;    // 迭代次数累加1

            int prevSize = lNodes.size();   // 当前节点总数，其是否变换会作为迭代终止的判断条件之一

            lit = lNodes.begin();   // 把迭代器指向当前lNodes的第一个元素

            int nToExpand = 0;      // 累加变量，包含特征个数大于1的节点个数

            vSizeAndPointerToNode.clear();  // 清空

            // 开始正式迭代、遍历lNodes中的节点
            while (lit != lNodes.end())
            {
                // 前面说过了，如果节点的bNoMore为true也就是说不可再分，那么直接跳过
                // 但别忘了把迭代器加一下
                if (lit->bNoMore)
                {
                    // If node only contains one point do not subdivide and continue
                    lit++;
                    continue;
                }
                // 如果当前节点的bNoMore为false，也就说明还可以继续分割，则进行下面的操作
                else
                {
                    // If more than one point, subdivide
                    // 新建四个临时节点变量，对应拆分的4个子节点
                    ExtractorNode n1, n2, n3, n4;
                    // 根据迭代器利用->操作符调用当前节点的成员函数DivideNode
                    // 由于函数参数是引用传递，因此它的输出值就是这四个子节点
                    // 这个函数做了哪些事情需要清楚，简单来说是三件事
                    // 1.根据父节点，对4个子节点的坐标范围进行了计算，4个子节点按照行优先顺序排列1 2;3 4
                    // 2.根据父节点特征点的x、y坐标，将各个特征点放到所属的子节点中
                    // 3.判断子节点是否为叶子节点(不可再分)，如果是，bNoMore设为True
                    // 最后需要说明的是这个函数并没有对父节点做任何操作，只是新建了4个覆盖父节点的子节点
                    // 因此需要在后续的操作中删除它们的父节点
                    lit->DivideNode(n1, n2, n3, n4);

                    // Add childs if they contain points
                    // 这里分别对拆分好的4个子节点依次进行了操作

                    // 如果子节点1中的特征点个数不为空，则执行下列操作
                    if (n1.vKeys.size() > 0)
                    {
                        // 将当前n1子节点添加到lNodes中
                        // 注意push_front是将元素添加到序列的最前面，而push_back则是添加到末尾
                        // 另外需要注意的是只有当节点包含的特征不为空时才会添加到lNodes
                        // 换句话说就是lNodes中的每个节点都至少包含一个或以上的特征点
                        // 若lNodes的长度等于期望特征数N，则实际提取到的特征点个数应该是大于等于N的
                        lNodes.push_front(n1);
                        // 进一步判断，如果当前节点包含的特征点多于1的话，也就说非叶子节点，bNoMore为false，则执行下面操作
                        if (n1.vKeys.size() > 1)
                        {
                            nToExpand++;    // 累加变量+1，只有当前节点包含特征点个数大于1才累加
                            // 这里利用make_pair函数构造了一个pair放到了vSizeAndPointerToNode中
                            // size就是当前节点的特征点个数，
                            // 由于上面将当前节点放到了lNodes的最前面，所以当前节点就是lNodes的第一个元素
                            // 由于定义的类型是ExtractorNode的指针，因此前面要加个取地址符
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                            // 这里的操作是把指向lNodes的第一个元素(也就是当前节点)的迭代器赋给lNodes中第一个元素(当前节点)的lit成员变量(别忘了节点中是有个迭代器成员变量的)
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    // 下面的n2,n3,n4都和上面的n1一样，这里就不再赘述了
                    if (n2.vKeys.size() > 0)
                    {
                        lNodes.push_front(n2);
                        if (n2.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n3.vKeys.size() > 0)
                    {
                        lNodes.push_front(n3);
                        if (n3.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n4.vKeys.size() > 0)
                    {
                        lNodes.push_front(n4);
                        if (n4.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    // 如果还记得在前面lit->DivideNode那行注释说的话的话，这里就是把父节点删掉的操作
                    // erase函数在前面也介绍过了，不仅会删除当前元素，还会返回指向下一个元素的迭代器
                    // 所以这里也就不再需要手动累加了
                    lit = lNodes.erase(lit);
                    continue;
                }
            }

            // 经过上面的操作，lNodes中的所有节点就都被遍历被迭代拆分了
            // 但别忘了这是在另一个更大的while循环里，
            // 直到目前为止我们并没有更改终止条件bFinish
            // 下面就是是否终止迭代的一系列判断与操作

            // Finish if there are more nodes than required features
            // or all nodes contain just one point
            // 如果lNodes中节点的个数大于期望提取的特征数量N(别忘了，这是函数传入的参数之一)
            // 或者lNodes中节点的个数和上次迭代一样，也就是说拆分不动了
            // 以上两种情况就停止迭代，bFinish设为true
            if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize)
            {
                bFinish = true;
            }
            // 而如果当前节点总数加上3倍nToExpand大于N，则执行下面操作
            // 这里体现的就是nToExpand的作用，上面说了nToExpand表示的是经过迭代后包含特征点数量大于1的节点的个数
            // 如果说当前节点总数加上3倍nToExpand大于N，则说明当前分割结果不够“干净”、“均匀”，还包含了大量特征多余1的节点
            // 至于说为什么是3，暂时还不太清楚
            else if (((int)lNodes.size() + nToExpand * 3) > N)
            {
                // 如果进入到这里，目前bFinish还为false所以会继续执行下面的代码，这其实和上面的是类似的
                while (!bFinish)
                {

                    prevSize = lNodes.size();   // 将当前的lNodes个数赋给prevSize

                    // 将当前的vSizeAndPointerToNode赋给vPrevSizeAndPointerToNode做个记录
                    vector<pair<int, ExtractorNode *>> vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                    vSizeAndPointerToNode.clear();  // 然后清空当前vector

                    // 对vPrevSizeAndPointerToNode做了个排序，默认为升序
                    // 另外排序默认是按照pair的第一个元素进行，且排序直接修改原始数据，没有另外的返回值
                    // 这样排完以后vPrevSizeAndPointerToNode中的元素是按照节点所包含的特征点个数按照从小到大的顺序排序
                    // 越往后节点包含的特征点个数越多，换句话说就是越可以继续拆分，直到一个节点包含一个特征点
                    sort(vPrevSizeAndPointerToNode.begin(), vPrevSizeAndPointerToNode.end());
                    // for循环遍历其中的每个元素，从后往前，原因在上一行已经说了
                    for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--)
                    {
                        // 循环里的操作其实和前面是一样的
                        ExtractorNode n1, n2, n3, n4;
                        vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3, n4);

                        // Add childs if they contain points
                        if (n1.vKeys.size() > 0)
                        {
                            lNodes.push_front(n1);
                            if (n1.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n2.vKeys.size() > 0)
                        {
                            lNodes.push_front(n2);
                            if (n2.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n3.vKeys.size() > 0)
                        {
                            lNodes.push_front(n3);
                            if (n3.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n4.vKeys.size() > 0)
                        {
                            lNodes.push_front(n4);
                            if (n4.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }

                        // 添加完子节点以后还是别忘了把父节点删掉
                        lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                        // 如果当前节点个数大于期望值N，就直接结束for循环
                        if ((int)lNodes.size() >= N)
                            break;
                    }

                    // 判断条件和前面一样，满足的话bFinish设为true
                    if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize)
                        bFinish = true;
                }
            }
        }

        // Retain the best point in each node
        // 建立vector用于保存结果
        vector<cv::KeyPoint> vResultKeys;
        vResultKeys.reserve(nfeatures);
        // 循环遍历每个节点
        for (list<ExtractorNode>::iterator lit = lNodes.begin(); lit != lNodes.end(); lit++)
        {
            // 获取到节点所包含的所有特征点
            vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
            // 获取到vector的第一个元素
            // 这里并不需要担心获取为空的问题，因为前面说了，能够加到lNodes的节点至少都包含了一个特征点
            cv::KeyPoint *pKP = &vNodeKeys[0];
            // 先将这个特征点的相应值作为最大值
            // 这里简单说一下response是OpenCV中KeyPoint的属性之一，
            // 可以理解为这个值越高说明特征点的质量越好
            float maxResponse = pKP->response;

            // 如果说当前节点包含多个特征点，
            // 那就遍历所有特征点，找到最大响应值以及其对应的特征点
            for (size_t k = 1; k < vNodeKeys.size(); k++)
            {
                // 响应值比较，比较好懂
                if (vNodeKeys[k].response > maxResponse)
                {
                    pKP = &vNodeKeys[k];
                    maxResponse = vNodeKeys[k].response;
                }
            }

            // 最后将这个有最大响应值的特征点添加到vResultKeys中
            vResultKeys.push_back(*pKP);
        }

        // 终于返回了
        // 通过上面这个循环也可以看出，最后每个节点只选取一个特征点
        // 所以提取的特征点数量和节点数量是相等的
        return vResultKeys;
    }

    void ORBextractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint>> &allKeypoints)
    {
        // allKeypoints每一个元素是一个vector，存放的是这一层提取到的特征点
        allKeypoints.resize(nlevels);

        // 每一个小格网的大小，这里宽高都是30个像素
        const float W = 30;

        // 对每一层循环进行特征点提取
        for (int level = 0; level < nlevels; ++level)
        {
            // 对于提取范围边界的一些计算
            const int minBorderX = EDGE_THRESHOLD - 3;                              // 16
            const int minBorderY = minBorderX;                                      // 16
            const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD + 3; // -16
            const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD + 3; // -16

            // 每一层都临时新建个vector变量用于存放特征点
            vector<cv::KeyPoint> vToDistributeKeys;
            // reserve函数用于申请内存
            // 在需要对大量数据进行处理的时候就要使用reserve主动分配内存以提升程序执行效率
            // 这里保留的是用户指定总特征个数的10倍的内存
            vToDistributeKeys.reserve(nfeatures * 10);

            // Compute the size of the image area for feature point extraction
            const float width = (maxBorderX - minBorderX);
            const float height = (maxBorderY - minBorderY);

            // Grid each level of the pyramid
            const int nCols = width / W;    // 网格列数
            const int nRows = height / W;   // 网格行数
            // Compute the width and the heigth of every grid
            const int wCell = ceil(width / nCols); // ceil: 返回大于或等于指定表达式的最小整数
            const int hCell = ceil(height / nRows);

            for (int i = 0; i < nRows; i++)
            {
                // 一系列格网坐标相关的判断与处理，不用过于深究
                const float iniY = minBorderY + i * hCell;
                float maxY = iniY + hCell + 6; // ??

                if (iniY >= maxBorderY - 3) // Does not meet the condition FAST-16-9(the edge=3)
                    continue;
                if (maxY > maxBorderY)
                    maxY = maxBorderY;

                // 为了保证中心像素周围的像素存在，所以设置了边界
                for (int j = 0; j < nCols; j++)
                {
                    // 一系列格网坐标相关的判断与处理，不用过于深究
                    const float iniX = minBorderX + j * wCell;
                    float maxX = iniX + wCell + 6;
                    if (iniX >= maxBorderX - 6) // ??
                        continue;
                    if (maxX > maxBorderX)
                        maxX = maxBorderX;

                    vector<cv::KeyPoint> vKeysCell;
                    // Use FAST method to extract keypoints
                    FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                         vKeysCell, iniThFAST, true);

                    // If keypoints is empty, use min threshold(7) of the pixel
                    if (vKeysCell.empty())
                    {
                        cout << "FAST threshold from 20 to 7" << endl;
                        FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                             vKeysCell, minThFAST, true);
                    }

                    // vKeysCell: The vector of keypoints
                    if (!vKeysCell.empty())
                    {
                        for (vector<cv::KeyPoint>::iterator vit = vKeysCell.begin(); vit != vKeysCell.end(); vit++)
                        {
                            // 将所有的关键点的坐标(x,y)设置为网格的左上角的坐标
                            (*vit).pt.x += j * wCell;
                            (*vit).pt.y += i * hCell;
                            vToDistributeKeys.push_back(*vit);
                        }
                    }
                }
            }

            // ??
            vector<KeyPoint> &keypoints = allKeypoints[level];
            keypoints.reserve(nfeatures);

            // 用八叉树来表示关键点信息
            keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                          minBorderY, maxBorderY, mnFeaturesPerLevel[level], level);

            const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

            // Add border to coordinates and scale information
            const int nkps = keypoints.size();
            for (int i = 0; i < nkps; i++)
            {
                keypoints[i].pt.x += minBorderX;
                keypoints[i].pt.y += minBorderY;
                keypoints[i].octave = level;
                keypoints[i].size = scaledPatchSize;
            }
        }

        // compute orientations
        for (int level = 0; level < nlevels; ++level)
            computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
    }

    void ORBextractor::ComputeKeyPointsOld(std::vector<std::vector<KeyPoint>> &allKeypoints)
    {
        allKeypoints.resize(nlevels);

        float imageRatio = (float)mvImagePyramid[0].cols / mvImagePyramid[0].rows;

        for (int level = 0; level < nlevels; ++level)
        {
            const int nDesiredFeatures = mnFeaturesPerLevel[level];

            const int levelCols = sqrt((float)nDesiredFeatures / (5 * imageRatio));
            const int levelRows = imageRatio * levelCols;

            const int minBorderX = EDGE_THRESHOLD;
            const int minBorderY = minBorderX;
            const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD;
            const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD;

            const int W = maxBorderX - minBorderX;
            const int H = maxBorderY - minBorderY;
            const int cellW = ceil((float)W / levelCols);
            const int cellH = ceil((float)H / levelRows);

            const int nCells = levelRows * levelCols;
            const int nfeaturesCell = ceil((float)nDesiredFeatures / nCells);

            vector<vector<vector<KeyPoint>>> cellKeyPoints(levelRows, vector<vector<KeyPoint>>(levelCols));

            vector<vector<int>> nToRetain(levelRows, vector<int>(levelCols, 0));
            vector<vector<int>> nTotal(levelRows, vector<int>(levelCols, 0));
            vector<vector<bool>> bNoMore(levelRows, vector<bool>(levelCols, false));
            vector<int> iniXCol(levelCols);
            vector<int> iniYRow(levelRows);
            int nNoMore = 0;
            int nToDistribute = 0;

            float hY = cellH + 6;

            for (int i = 0; i < levelRows; i++)
            {
                const float iniY = minBorderY + i * cellH - 3;
                iniYRow[i] = iniY;

                if (i == levelRows - 1)
                {
                    hY = maxBorderY + 3 - iniY;
                    if (hY <= 0)
                        continue;
                }

                float hX = cellW + 6;

                for (int j = 0; j < levelCols; j++)
                {
                    float iniX;

                    if (i == 0)
                    {
                        iniX = minBorderX + j * cellW - 3;
                        iniXCol[j] = iniX;
                    }
                    else
                    {
                        iniX = iniXCol[j];
                    }

                    if (j == levelCols - 1)
                    {
                        hX = maxBorderX + 3 - iniX;
                        if (hX <= 0)
                            continue;
                    }

                    Mat cellImage = mvImagePyramid[level].rowRange(iniY, iniY + hY).colRange(iniX, iniX + hX);

                    cellKeyPoints[i][j].reserve(nfeaturesCell * 5);

                    FAST(cellImage, cellKeyPoints[i][j], iniThFAST, true);

                    if (cellKeyPoints[i][j].size() <= 3)
                    {
                        cellKeyPoints[i][j].clear();

                        FAST(cellImage, cellKeyPoints[i][j], minThFAST, true);
                    }

                    const int nKeys = cellKeyPoints[i][j].size();
                    nTotal[i][j] = nKeys;

                    if (nKeys > nfeaturesCell)
                    {
                        nToRetain[i][j] = nfeaturesCell;
                        bNoMore[i][j] = false;
                    }
                    else
                    {
                        nToRetain[i][j] = nKeys;
                        nToDistribute += nfeaturesCell - nKeys;
                        bNoMore[i][j] = true;
                        nNoMore++;
                    }
                }
            }

            // Retain by score

            while (nToDistribute > 0 && nNoMore < nCells)
            {
                int nNewFeaturesCell = nfeaturesCell + ceil((float)nToDistribute / (nCells - nNoMore));
                nToDistribute = 0;

                for (int i = 0; i < levelRows; i++)
                {
                    for (int j = 0; j < levelCols; j++)
                    {
                        if (!bNoMore[i][j])
                        {
                            if (nTotal[i][j] > nNewFeaturesCell)
                            {
                                nToRetain[i][j] = nNewFeaturesCell;
                                bNoMore[i][j] = false;
                            }
                            else
                            {
                                nToRetain[i][j] = nTotal[i][j];
                                nToDistribute += nNewFeaturesCell - nTotal[i][j];
                                bNoMore[i][j] = true;
                                nNoMore++;
                            }
                        }
                    }
                }
            }

            vector<KeyPoint> &keypoints = allKeypoints[level];
            keypoints.reserve(nDesiredFeatures * 2);

            const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

            // Retain by score and transform coordinates
            for (int i = 0; i < levelRows; i++)
            {
                for (int j = 0; j < levelCols; j++)
                {
                    vector<KeyPoint> &keysCell = cellKeyPoints[i][j];
                    KeyPointsFilter::retainBest(keysCell, nToRetain[i][j]);
                    if ((int)keysCell.size() > nToRetain[i][j])
                        keysCell.resize(nToRetain[i][j]);

                    for (size_t k = 0, kend = keysCell.size(); k < kend; k++)
                    {
                        keysCell[k].pt.x += iniXCol[j];
                        keysCell[k].pt.y += iniYRow[i];
                        keysCell[k].octave = level;
                        keysCell[k].size = scaledPatchSize;
                        keypoints.push_back(keysCell[k]);
                    }
                }
            }

            if ((int)keypoints.size() > nDesiredFeatures)
            {
                KeyPointsFilter::retainBest(keypoints, nDesiredFeatures);
                keypoints.resize(nDesiredFeatures);
            }
        }

        // and compute orientations
        for (int level = 0; level < nlevels; ++level)
            computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
    }

    static void computeDescriptors(const Mat &image, vector<KeyPoint> &keypoints, Mat &descriptors,
                                   const vector<Point> &pattern)
    {
        // 这里其实也是一层包装，核心函数是computeOrbDescriptor
        // 这里相当于对传入的descriptors重新赋值，元素全部为0，
        // 行数为特征点个数，列数为32(描述子长度)，元素数据类型为8位无符号整型(最大为255)
        descriptors = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);

        for (size_t i = 0; i < keypoints.size(); i++)
            // 第一个参数是特征点，第二个参数是影像，第三个参数是Point对象，第四个参数就是待填充的描述子
            // 注意一下传入的描述子的类型与大小，它其实是一个uchar类型的指针，可以根据索引获取到第i行的所有元素，因此可以说大小为1x32
            computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
    }

    // Extractor keypoints for new frame
    // mask: 掩膜，这里没有用到
    void ORBextractor::operator()(InputArray _image, InputArray _mask, vector<KeyPoint> &_keypoints,
                                  OutputArray _descriptors)
    {
        if (_image.empty())
            return;

        Mat image = _image.getMat();
        // 如果输入的图像不是灰度，中断程序
        assert(image.type() == CV_8UC1);

        // Pre-compute the scale pyramid
        // 它的返回值是直接返回给了ORB提取对象的public成员变量mvImagePyramid，所以这里看起来没有返回值
        ComputePyramid(image);

        // allKeypoints有很多层，每一层有很多个特征点，所以要用两个vector嵌套
        vector<vector<KeyPoint>> allKeypoints;
        // 这里为了分布均匀，采用OctTree分割图像
        ComputeKeyPointsOctTree(allKeypoints);
        // ComputeKeyPointsOld(allKeypoints);

        // 开始特征描述相关操作
        Mat descriptors;

        // 统计各层提取到的特征点的总和

        int nkeypoints = 0;
        for (int level = 0; level < nlevels; ++level)
        {
            nkeypoints += (int)allKeypoints[level].size();
            cout << "pyramid level " << level << "'s keypoints: " << allKeypoints[level].size() << endl;
        }
        cout << "nkeypoints: " << nkeypoints << endl;

        // 如果一个特征点都没有提取到
        if (nkeypoints == 0)
            // 就把传入的_descriptors释放掉
            _descriptors.release();
        else
        {
            // 否则就创建一个Mat用于描述
            // 这个Mat有nkeypoints行，32列，元素数据类型为CV_8U
            // CV_8U: 无8位无符号整数 0-255
            // 也就是说每个特征点由32个CV_8U的数字描述
            _descriptors.create(nkeypoints, 32, CV_8U);
            // 将传入的描述符先付给新建的descriptors方便操作
            descriptors = _descriptors.getMat();
        }

        // 先将传入的keypoints清空，防止出错
        _keypoints.clear();
        _keypoints.reserve(nkeypoints); // 设置长度

        int offset = 0; // 描述子的偏移量，用于在迭代计算描述子时找到正确的位置
        // 按层数依次遍历特征点
        for (int level = 0; level < nlevels; ++level)
        {
            // 根据索引获取每一层特征点的vector
            vector<KeyPoint> &keypoints = allKeypoints[level];
            int nkeypointsLevel = (int)keypoints.size(); // 当前层特征点的数量

            // 如果当前层特征点数为0则跳过循环
            if (nkeypointsLevel == 0)
                continue;

            // preprocess the resized image
            // 先获取当前层对应的图像内容，赋给workingMat方便后续处理
            Mat workingMat = mvImagePyramid[level].clone();
            // 调用OpenCV的高斯模糊函数对图像进行模糊
            // (输入，输出，高斯卷积核大小，X、Y方向上的方差sigma，边缘扩展的方式)
            GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

            // Compute the descriptors
            // 获取当前层所包含特征点数量的行数作为描述子待填充的Mat
            Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
            // 传入图像、特征点、带填充的描述子、pattern
            computeDescriptors(workingMat, keypoints, desc, pattern);

            // 将偏移量增加nkeypointsLevel
            offset += nkeypointsLevel;

            // Scale keypoint coordinates
            // 如果不是金字塔的第一层（没有缩放），就对特征点坐标乘上缩放因子
            if (level != 0)
            {
                float scale = mvScaleFactor[level]; // getScale(level, firstLevel, scaleFactor);
                for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                                                keypointEnd = keypoints.end();
                     keypoint != keypointEnd; ++keypoint)
                    keypoint->pt *= scale; // scale: 逆尺度；乘完得到在原始大小图像中的坐标
            }
            // And add the keypoints to the output
            // 最后，将提取的特征点插入传入参数_keypoints的尾部
            // 但因为在上面已经清空了_keypoints，所以这里其实是不存在其它特征点的
            _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
        }
    }

    void ORBextractor::ComputePyramid(cv::Mat image)
    {
        // nlevels是成员变量，在构造函数中被初始化
        for (int level = 0; level < nlevels; ++level)
        {
            // 这里获取的是逆尺度，也就是说里面的尺度因子是小于1并且慢慢变小的
            // 用户指定的尺度因子大于1的情况下
            float scale = mvInvScaleFactor[level];
            // 根据尺度因子计算当前层的长宽尺寸
            Size sz(cvRound((float)image.cols * scale), cvRound((float)image.rows * scale));
            // 由于还涉及到扩边的问题，所以对图像长宽都加上一定数值
            Size wholeSize(sz.width + EDGE_THRESHOLD * 2, sz.height + EDGE_THRESHOLD * 2);
            // 新建两个Mat，temp类型和输入的image一致
            Mat temp(wholeSize, image.type()), masktemp;
            // 取temp的一定范围内的图像内容放到vector里
            // 对temp的修改也会影响到mvImagePyramid
            // 此操作不复制矩阵的数据，是O(1)操作
            mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

            // Compute the resized image
            if (level != 0)
            {
                // 根据上一层金字塔图像的大小调整当前层的图像
                resize(mvImagePyramid[level - 1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);

                // BORDER_REFLECT_101是以镜面对称的方式扩边，BORDER_ISOLATED是以0灰度填充
                // 它们两个混合使用的效果和单独使用reflect_101效果一样
                // 函数第一个参数是输入图像，第二个参数是输出图像，后面四个参数是上下左右方向扩展的距离，这里都是一样的
                // 每一层金字塔图像的大小是不一样的；
                // 图像金字塔从底层（原图像）到顶层是缩放的，即镜头拉远。这似乎违背SLAM的直觉
                // 但在Dual-SLAM中提到了，SLAM正序和倒序的结果没有太大的差别
                copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                               BORDER_REFLECT_101 + BORDER_ISOLATED);
            }
            else
            {
                // 将image扩边，返回给temp，四个方向大小相同都为EDGE_THRESHOLD，
                // 将原图通过插值(BORDER_REFLECT_101)的方式放大
                copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                               BORDER_REFLECT_101);
            }
        }
        // for (int level = 0; level < nlevels; ++level)
        // {
        //     std::cout << "====== Pyramid Size ======" << std::endl;
        //     std::cout << mvImagePyramid[level].size() << std::endl;
        //     std::cout << "=======Pyramid Size=======" << std::endl;
        // }
    }

} // namespace ORB_SLAM

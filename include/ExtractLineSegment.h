//
// Created by lan on 17-12-13.
//

#ifndef ORB_SLAM2_LINEFEATURE_H
#define ORB_SLAM2_LINEFEATURE_H

#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/line_descriptor/descriptor.hpp>
//#include <opencv2/features2d/features2d.hpp>
// Thirdparty中的#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>
//

#include <eigen3/Eigen/Core>
#include "auxiliar.h"

using namespace std;
using namespace cv;
//using namespace line_descriptor;      // Thirdparty中的

namespace ORB_SLAM2
{
    class LineSegment
    {
    public:
        // constructor
        LineSegment();

        ~LineSegment(){}

        // 提取线特征,计算线特征描述子
        void ExtractLineSegment(const Mat &img, vector<KeyLine> &keylines, Mat &ldesc, vector<Vector3d> &keylineFunctions, int scale = 1.2, int numOctaves = 1);

        // 线特征匹配
        void LineSegmentMatch(Mat &ldesc_1, Mat &ldesc_2);

        // 线特征的描述子距离中位值
        void lineDescriptorMAD();

        // 求线特征的观测线段和重投影线段的重合率
        double lineSegmentOverlap(double spl_obs, double epl_obs, double spl_proj, double epl_proj  );

    protected:
        vector<vector<DMatch>> line_matches;
        double nn_mad, nn12_mad;
    };
}


#endif //ORB_SLAM2_LINEFEATURE_H

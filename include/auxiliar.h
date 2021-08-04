//
// Created by lan on 17-12-18.
//

#pragma once

#include <iostream>

#include <cv.h>
#include <opencv2/features2d/features2d.hpp>
//#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>
//#include <opencv2/line_descriptor/descriptor.hpp>
using namespace cv;
using namespace cv::line_descriptor;
//using namespace line_descriptor;

#include <vector>
using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
using namespace Eigen;

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;

// 比较线特征距离的两种方式，自己添加的
struct compare_descriptor_by_NN_dist
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ( a[0].distance < b[0].distance);
    }
};

struct conpare_descriptor_by_NN12_dist
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ((a[1].distance - a[0].distance) > (b[1].distance - b[0].distance));
    }
};

// 按描述子之间距离的从小到大方式排序
struct sort_descriptor_by_queryIdx
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ( a[0].queryIdx < b[0].queryIdx );
    }
};

struct sort_lines_by_response
{
    inline bool operator()(const KeyLine& a, const KeyLine& b){
        return ( a.response > b.response );
    }
};

inline Mat SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<  0, -v.at<float>(2), v.at<float>(1),
                        v.at<float>(2),               0,-v.at<float>(0),
                       -v.at<float>(1),  v.at<float>(0),             0);
}

/**
 * @brief 求一个vector数组的中位数绝对偏差MAD
 * 中位数绝对偏差MAD——median absolute deviation, 是单变量数据集中样本差异性的稳健度量。
 * MAD是一个健壮的统计量，对于数据集中异常值的处理比标准差更具有弹性，可以大大减少异常值对于数据集的影响
 * 对于单变量数据集 X={X1,X2,X3,...,Xn}, MAD的计算公式为：MAD(X)=median(|Xi-median(X)|)
 * @param residues
 * @return
 */
inline double vector_mad(vector<double> residues)
{
    if(residues.size()!=0)
    {
        // Return the standard deviation of vector with MAD estimation
        int n_samples = residues.size();
        sort(residues.begin(), residues.end());
        double median = residues[n_samples/2];
        for(int i=0; i<n_samples; i++)
            residues[i] = fabs(residues[i]-median);
        sort(residues.begin(), residues.end());
        double MAD = residues[n_samples/2];
        return 1.4826*MAD;
    } else
        return 0.0;
}

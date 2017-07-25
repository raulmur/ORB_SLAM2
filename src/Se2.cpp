#include "Se2.h"
#include <cmath>

Se2::Se2(){}
Se2::Se2(float _x, float _y ,float _theta):
    x(_x), y(_y), theta(normalize_angle(_theta)){}
Se2::~Se2(){}

Se2 Se2::inv() const
{
    float c = std::cos(theta);
    float s = std::sin(theta);
    return Se2(-c*x-s*y, s*x-c*y, -theta);
}

Se2 Se2::operator +(const Se2& that) const{
    float c = std::cos(theta);
    float s = std::sin(theta);
    float _x = x + that.x*c - that.y*s;
    float _y = y + that.x*s + that.y*c;
    float _theta = normalize_angle(theta + that.theta);
    return Se2(_x, _y, _theta);
}

// Same as: that.inv() + *this
Se2 Se2::operator -(const Se2& that) const{
    float dx = x - that.x;
    float dy = y - that.y;
    float dth = normalize_angle(theta - that.theta);

    float c = std::cos(that.theta);
    float s = std::sin(that.theta);
    return Se2(c*dx+s*dy, -s*dx+c*dy, dth);
}

cv::Mat Se2::toCvSE3() const
{
    float c = cos(theta);
    float s = sin(theta);

    return (cv::Mat_<float>(4,4) <<
            c,-s, 0, x,
            s, c, 0, y,
            0, 0, 1, 0,
            0, 0, 0, 1);
}

Se2& Se2::fromCvSE3(const cv::Mat &mat)
{
    float yaw = std::atan2(mat.at<float>(1,0), mat.at<float>(0,0));
    theta = normalize_angle(yaw);
    x = mat.at<float>(0,3);
    y = mat.at<float>(1,3);
    return *this;
}

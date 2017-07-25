#ifndef UTIL_SE2_H
#define UTIL_SE2_H
#include <opencv2/core/core.hpp>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


class  Se2{
public:
    float x;
    float y;
    float theta;
    Se2();
    Se2(float _x, float _y ,float _theta);
    ~Se2();
    Se2 inv() const;
    Se2 operator -(const Se2& that) const;
    Se2 operator +(const Se2& that) const;
    cv::Mat toCvSE3() const;
    Se2& fromCvSE3(const cv::Mat& mat);
};


inline double normalize_angle(double theta)
{
  if (theta >= -M_PI && theta < M_PI)
    return theta;

  double multiplier = floor(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}



#endif // SE2_H

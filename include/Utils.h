#ifndef UTILS_H
#define UTILS_H

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui.hpp>

namespace Utils{
	//find homography matrix from tranform matrix and camera matrix
	inline cv::Mat findHomography(const cv::Mat &camera, const cv::Mat &rotation, const cv::Mat translation){
		cv::Mat temp = cv::Mat::eye(3, 3, CV_32F);
		cv::Mat temp2 = rotation.t()*translation;
		for (int i = 0; i < 3; ++i){
			temp.at<float>(i, 2) = temp2.at<float>(i);
		}
		return camera*rotation*temp;
	}
}












#endif
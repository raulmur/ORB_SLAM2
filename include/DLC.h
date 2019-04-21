#ifndef DLC_H
#define DLC_H

#include<array>
#include<cmath>
#include <opencv2/opencv.hpp>
#include "deeplcd.h"

namespace DLC {

	const int hog_length = 3648;
	typedef  std::array<double, hog_length> HOGdescriptor;

	double ComputeL2Norm(HOGdescriptor h1, HOGdescriptor h2);

	double ComputeL1Norm(HOGdescriptor h1, HOGdescriptor h2);

	HOGdescriptor ExtractHOG(const cv::Mat &im);
	// possible extensions - compute KL divergence, Bhattacharyya distance
}

#endif //__DLC_H__
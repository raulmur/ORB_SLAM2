#ifndef DLC_H
#define DLC_H

#include<array>
#include<cmath>
#include <opencv2/opencv.hpp>


const int hog_length = 3648;
typedef  std::array<double, hog_length> HOGdescriptor;

namespace DLC {
	
	double ComputeL2Norm(HOGdescriptor h1, HOGdescriptor h2)
	{
		double distance = 0.0;
		for(
			int i = 0;
			i < hog_length;
			i++ )
		{
			distance += pow( (h1[i] - h2[i]), 2);
		}
		return distance;
	}

	double ComputeL1Norm(HOGdescriptor h1, HOGdescriptor h2)
	{
		double distance = 0.0;
		for(
			int i = 0;
			i < hog_length;
			i++ )
		{
			distance += pow( (h1[i] - h2[i]), 2);
		}
		return distance;
	}

	HOGdescriptor ExtractHOG(const cv::Mat &im)
	{
		// run HOG classifier
		HOGdescriptor imDescriptor = {0};
	}
	// possible extensions - compute KL divergence, Bhattacharyya distance
}

#endif //__DLC_H__
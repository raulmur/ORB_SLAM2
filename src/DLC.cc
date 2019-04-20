#include "DLC.h"

double DLC::ComputeL2Norm(HOGdescriptor h1, HOGdescriptor h2)
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

double DLC::ComputeL1Norm(HOGdescriptor h1, HOGdescriptor h2)
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

DLC::HOGdescriptor DLC::ExtractHOG(const cv::Mat &im)
{
	// run HOG classifier
	HOGdescriptor imDescriptor = {0};
	return imDescriptor;
}
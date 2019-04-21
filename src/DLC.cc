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
	// deeplcd repo - demo.cpp
	deeplcd::DeepLCD test_lcd;
	cv::Mat im_current(im);
	cv::Size sz(160, 120);

	cv::cvtColor(im_current, im_current, cv::COLOR_BGR2GRAY); // convert to grayscale
	cv::resize(im_current, im_current, sz);

	test_lcd.add(im_current)
	deeplcd::query_result q = test_lcd.query(im_current, 0);

	// Okay now we have a database of descriptors, lets see if we can match them now
	int i = 0;

	// run HOG classifier
	HOGdescriptor imDescriptor = {0};
	return imDescriptor;
}

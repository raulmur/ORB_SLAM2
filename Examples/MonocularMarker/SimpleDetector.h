#ifndef SIMPLEDETECTOR_H
#define SIMPLEDETECTOR_H

#include <MarkerDetector.h>

class SimpleDetector :public MarkerDetector{
public:
	SimpleDetector(const string &strSettingsFile, int minFeatures = 20);

	int detect(const cv::Mat &im, const std::vector<cv::KeyPoint> &keypoints, cv::Mat &rmat, cv::Mat &tvec, std::vector<bool> &mask, std::vector<cv::Point3f> &worldPos) override;

	static void makeThreshold(const cv::Mat &in, cv::Mat &out);
	static void detectRectangles(cv::Mat &thresImages, std::vector<std::vector<cv::Point> > &rectangles);
	static cv::Mat warp(const cv::Mat &source, const std::vector<cv::Point> &rect, const cv::Size &size = cv::Size(56, 56));

	static void reOrder(std::vector<cv::Point> &points);

	static float _minSize, _maxSize;
	static int _minSize_pix;

	static bool contains(const std::vector<cv::Point2f> &square, const cv::Point2f &point);
};








#endif
/**
* This file sucks.
*
* Copyright (C) 2014-2016 Ra¨²l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PLAINPAPERDETECTOR_H
#define PLAINPAPERDETECTOR_H

#include <MarkerDetector.h>

class PlainPaperDetector :public MarkerDetector{
public:
	PlainPaperDetector(const string &strSettingsFile, int minFeatures = 4);

	int detect(const cv::Mat &im, const std::vector<cv::KeyPoint> &keypoints, cv::Mat &rmat, cv::Mat &tvec, std::vector<bool> &mask, std::vector<cv::Point3f> &worldPos) override;

	static void makeThreshold(const cv::Mat &in, cv::Mat &out);
	static void detectRectangles(cv::Mat &thresImages, std::vector<std::vector<cv::Point> > &rectangles);
	static cv::Mat warp(const cv::Mat &source, const std::vector<cv::Point> &rect, const cv::Size &size = cv::Size(63, 89));

	static int adapt(const std::vector<cv::KeyPoint> &keypoints, std::vector<cv::Point> &rectangle, std::vector<bool> &mask);
	static void reOrder(std::vector<cv::Point> &points);

};








#endif
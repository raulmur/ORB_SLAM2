#ifndef MARKER_H
#define MARKER_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <memory>
#include <bitset>

//#include <QImage>

class Marker5
{
public:
	Marker5(uchar id, uchar scale);

	static std::unique_ptr<Marker5> detect(cv::Mat &in, std::vector<cv::Point> &square);

	static std::vector<cv::Point3f> getObjectPoints(int scale);
	static std::vector<cv::Point2f> getPlannerPoints(int scale);

	cv::Mat toMat();
	static const cv::Size size;
	std::vector<cv::Point2f> points;

	int getId() const;
	int getScale() const;

#ifdef QIMAGE_H
	QImage toImage();
#endif

protected:
	static void rotateBits(std::bitset<7> &id, std::bitset<7> &scale, std::bitset<7> &check, int n);

private:
	std::bitset<7> id;
	//Printing on A4 paper, scale = 0 is designed to be 140 pixels(of 1240, 150ppi), or 23.71mm of 210mm.
	//pixel width of marker will be scale*7 + 140
	std::bitset<7> scale;
};

#endif // MARKER_H

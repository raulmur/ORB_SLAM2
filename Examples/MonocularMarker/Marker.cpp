#include "marker.h"

const cv::Size Marker5::size = cv::Size(56, 56);

Marker5::Marker5(uchar id, uchar scale) :id(id), scale(scale)
{

}


std::unique_ptr<Marker5> Marker5::detect(cv::Mat &in, std::vector<cv::Point> &square){
	cv::Mat gray = in.clone();
	if (in.channels() == 3)
		cv::cvtColor(in, gray, cv::COLOR_BGR2GRAY);
	else if (in.channels() == 4)
		cv::cvtColor(in, gray, cv::COLOR_BGRA2GRAY);
	cv::threshold(gray, gray, 127, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	const int unit = 8;
	const int total_num = 7;
	const int num = 5;

	std::unique_ptr<Marker5> ret = std::make_unique<Marker5>(0, 0);


	//check rotation and reject non-markers
	for (int x = 0; x<total_num; ++x){
		int step = total_num - 1;
		if (x == 0 || x == total_num - 1)
			step = 1;
		for (int y = 0; y<total_num; y += step){
			if (cv::countNonZero(gray(cv::Rect(x*unit, y*unit, unit, unit)))>(unit*unit) / 2)
				return std::unique_ptr<Marker5>();
		}
	}

	cv::Mat inner = gray(cv::Rect(unit, unit, num*unit, num*unit));
	std::bitset<4> corners;
	corners[0] = cv::countNonZero(inner(cv::Rect(0, 0, unit, unit)))>(unit*unit) / 2;
	corners[1] = cv::countNonZero(inner(cv::Rect((num - 1)*unit, 0, unit, unit)))>(unit*unit) / 2;
	corners[2] = cv::countNonZero(inner(cv::Rect((num - 1)*unit, (num - 1)*unit, unit, unit)))>(unit*unit) / 2;
	corners[3] = cv::countNonZero(inner(cv::Rect(0, (num - 1)*unit, unit, unit)))>(unit*unit) / 2;


	if (corners.count() != 1){
		return std::unique_ptr<Marker5>();
	}

	int startIndex = 0;
	while (!corners[0]) {
		corners >>= 1;
		corners[3] = false;
		startIndex += 1;
	}
	for (int i = 0; i<4; ++i){
		ret->points.push_back(cv::Point2f(square[(startIndex + i) % 4].x, square[(startIndex + i) % 4].y));
	}


	//get bits
	std::bitset<7> tid, tscale, tcheck;
	//id
	tid[0] = cv::countNonZero(inner(cv::Rect(unit, 0, unit, unit)))>(unit*unit) / 2;
	tid[1] = cv::countNonZero(inner(cv::Rect(2 * unit, 0, unit, unit)))>(unit*unit) / 2;
	tid[2] = cv::countNonZero(inner(cv::Rect(3 * unit, 0, unit, unit)))>(unit*unit) / 2;
	tid[3] = cv::countNonZero(inner(cv::Rect(0, unit, unit, unit)))>(unit*unit) / 2;
	tid[4] = cv::countNonZero(inner(cv::Rect(unit, unit, unit, unit)))>(unit*unit) / 2;
	tid[5] = cv::countNonZero(inner(cv::Rect(2 * unit, unit, unit, unit)))>(unit*unit) / 2;
	tid[6] = cv::countNonZero(inner(cv::Rect(3 * unit, unit, unit, unit)))>(unit*unit) / 2;
	//scale
	tcheck[0] = cv::countNonZero(inner(cv::Rect(4 * unit, unit, unit, unit)))>(unit*unit) / 2;
	tcheck[1] = cv::countNonZero(inner(cv::Rect(0, 2 * unit, unit, unit)))>(unit*unit) / 2;
	tcheck[2] = cv::countNonZero(inner(cv::Rect(unit, 2 * unit, unit, unit)))>(unit*unit) / 2;
	tcheck[3] = cv::countNonZero(inner(cv::Rect(2 * unit, 2 * unit, unit, unit)))>(unit*unit) / 2;
	tcheck[4] = cv::countNonZero(inner(cv::Rect(3 * unit, 2 * unit, unit, unit)))>(unit*unit) / 2;
	tcheck[5] = cv::countNonZero(inner(cv::Rect(4 * unit, 2 * unit, unit, unit)))>(unit*unit) / 2;
	tcheck[6] = cv::countNonZero(inner(cv::Rect(0, 3 * unit, unit, unit)))>(unit*unit) / 2;
	//check
	tscale[0] = cv::countNonZero(inner(cv::Rect(unit, 3 * unit, unit, unit)))>(unit*unit) / 2;
	tscale[1] = cv::countNonZero(inner(cv::Rect(2 * unit, 3 * unit, unit, unit)))>(unit*unit) / 2;
	tscale[2] = cv::countNonZero(inner(cv::Rect(3 * unit, 3 * unit, unit, unit)))>(unit*unit) / 2;
	tscale[3] = cv::countNonZero(inner(cv::Rect(4 * unit, 3 * unit, unit, unit)))>(unit*unit) / 2;
	tscale[4] = cv::countNonZero(inner(cv::Rect(unit, 4 * unit, unit, unit)))>(unit*unit) / 2;
	tscale[5] = cv::countNonZero(inner(cv::Rect(2 * unit, 4 * unit, unit, unit)))>(unit*unit) / 2;
	tscale[6] = cv::countNonZero(inner(cv::Rect(3 * unit, 4 * unit, unit, unit)))>(unit*unit) / 2;

	rotateBits(tid, tscale, tcheck, startIndex);

	if ((tid^tscale) == tcheck){
		ret->id = tid;
		ret->scale = tscale;
		return ret;
	}

	return std::unique_ptr<Marker5>();



}

void Marker5::rotateBits(std::bitset<7> &id, std::bitset<7> &scale, std::bitset<7> &check, int n){
	std::bitset<7> tid, tscale, tcheck;
	switch (n) {
	case 0:
		return;
	case 1:
		tid[0] = check[0];
		tid[1] = check[5];
		tid[2] = scale[3];
		tid[3] = id[2];
		tid[4] = id[6];
		tid[5] = check[4];
		tid[6] = scale[2];

		tscale[0] = id[4];
		tscale[1] = check[2];
		tscale[2] = scale[0];
		tscale[3] = scale[4];
		tscale[4] = id[3];
		tscale[5] = check[1];
		tscale[6] = check[6];

		tcheck[0] = scale[6];
		tcheck[1] = id[1];
		tcheck[2] = id[5];
		tcheck[3] = check[3];
		tcheck[4] = scale[1];
		tcheck[5] = scale[5];
		tcheck[6] = id[0];

		id = tid;
		scale = tscale;
		check = tcheck;
		break;
	case 2:
		for (int i = 0; i<7; ++i){
			tid[i] = scale[6 - i];
			tscale[i] = id[6 - i];
			tcheck[i] = check[6 - i];
		}

		id = tid;
		scale = tscale;
		check = tcheck;
		break;
	case 3:
		tid[0] = check[6];
		tid[1] = check[1];
		tid[2] = id[3];
		tid[3] = scale[4];
		tid[4] = scale[0];
		tid[5] = check[2];
		tid[6] = id[4];

		tscale[0] = scale[2];
		tscale[1] = check[4];
		tscale[2] = id[6];
		tscale[3] = id[2];
		tscale[4] = scale[3];
		tscale[5] = check[5];
		tscale[6] = check[0];

		tcheck[0] = id[0];
		tcheck[1] = scale[5];
		tcheck[2] = scale[1];
		tcheck[3] = check[3];
		tcheck[4] = id[5];
		tcheck[5] = id[1];
		tcheck[6] = scale[6];

		id = tid;
		scale = tscale;
		check = tcheck;
		break;
	default:
		std::cout << "error! nRotate>=4" << std::endl;
		break;
	}
	return;
}

cv::Mat Marker5::toMat(){
	cv::Mat temp = cv::Mat::zeros(7, 7, CV_8UC1);
	std::bitset<7> check = id^scale;

	cv::Mat temp2 = temp(cv::Rect(1, 1, 5, 5));
	temp2.at<uchar>(0, 0) = 255;
	temp2.at<uchar>(0, 1) = id[0] ? 255 : 0;
	temp2.at<uchar>(0, 2) = id[1] ? 255 : 0;
	temp2.at<uchar>(0, 3) = id[2] ? 255 : 0;
	temp2.at<uchar>(0, 4) = 0;
	temp2.at<uchar>(1, 0) = id[3] ? 255 : 0;
	temp2.at<uchar>(1, 1) = id[4] ? 255 : 0;
	temp2.at<uchar>(1, 2) = id[5] ? 255 : 0;
	temp2.at<uchar>(1, 3) = id[6] ? 255 : 0;

	temp2.at<uchar>(1, 4) = check[0] ? 255 : 0;
	temp2.at<uchar>(2, 0) = check[1] ? 255 : 0;
	temp2.at<uchar>(2, 1) = check[2] ? 255 : 0;
	temp2.at<uchar>(2, 2) = check[3] ? 255 : 0;
	temp2.at<uchar>(2, 3) = check[4] ? 255 : 0;
	temp2.at<uchar>(2, 4) = check[5] ? 255 : 0;
	temp2.at<uchar>(3, 0) = check[6] ? 255 : 0;

	temp2.at<uchar>(3, 1) = scale[0] ? 255 : 0;
	temp2.at<uchar>(3, 2) = scale[1] ? 255 : 0;
	temp2.at<uchar>(3, 3) = scale[2] ? 255 : 0;
	temp2.at<uchar>(3, 4) = scale[3] ? 255 : 0;
	temp2.at<uchar>(4, 0) = 0;
	temp2.at<uchar>(4, 1) = scale[4] ? 255 : 0;
	temp2.at<uchar>(4, 2) = scale[5] ? 255 : 0;
	temp2.at<uchar>(4, 3) = scale[6] ? 255 : 0;
	temp2.at<uchar>(4, 4) = 0;

	cv::Mat ret;
	cv::resize(temp, ret, cv::Size(56, 56), 0, 0, cv::INTER_NEAREST);
	return ret;
}

int Marker5::getId() const{
	return static_cast<int>(id.to_ulong());
}

int Marker5::getScale() const{
	return static_cast<int>(scale.to_ulong());
}

std::vector<cv::Point3f> Marker5::getObjectPoints(int scale){
	std::vector<cv::Point3f> origins;
	origins.push_back(cv::Point3f(-(7 * scale + 140), -(7 * scale + 140), 0));
	origins.push_back(cv::Point3f(7 * scale + 140, -(7 * scale + 140), 0));
	origins.push_back(cv::Point3f(7 * scale + 140, 7 * scale + 140, 0));
	origins.push_back(cv::Point3f(-(7*scale+140),7*scale+140,0));
	return origins;
}

std::vector<cv::Point2f> Marker5::getPlannerPoints(int scale){
	std::vector<cv::Point2f> origins;
	origins.push_back(cv::Point2f(-(7 * scale + 140), -(7 * scale + 140)));
	origins.push_back(cv::Point2f(7 * scale + 140, -(7 * scale + 140)));
	origins.push_back(cv::Point2f(7 * scale + 140, 7 * scale + 140));
	origins.push_back(cv::Point2f(-(7*scale+140),7*scale+140));
	return origins;
}


#ifdef QIMAGE_H
QImage Marker5::toImage(){
	cv::Mat mat = this->toMat();
	QImage image(mat.data, 56, 56, QImage::Format_Grayscale8);

	return image.copy();
}

#endif

#include "SimpleDetector.h"
#include "Marker.h"
typedef unsigned int uint;
typedef Marker5 Marker;

float SimpleDetector::_maxSize = 0.95f;
float SimpleDetector::_minSize = 0.04f;
int SimpleDetector::_minSize_pix = 25;

SimpleDetector::SimpleDetector(const string &strSettingsFile, int minFeatures) :MarkerDetector(strSettingsFile, minFeatures){

}

int SimpleDetector::detect(const cv::Mat &im, const std::vector<cv::KeyPoint> &keypoints, cv::Mat &rmat, cv::Mat &tvec, std::vector<bool> &mask, std::vector<cv::Point3f> &worldPos){
	std::vector<std::unique_ptr<Marker>> markers;
	std::vector<std::vector<cv::Point>> squares;
	std::vector<cv::Mat> transforms;
	int ret = 0;

	cv::Mat thres;
	makeThreshold(im, thres);
	detectRectangles(thres, squares);

	for (auto square:squares){
		reOrder(square);
		cv::Mat raw = warp(im, square, Marker::size);
		auto marker = Marker::detect(raw, square);
		if (marker){
			markers.push_back(std::move(marker));
		}
	}

	int maxId = -1;
	uint maxIndex = -1;
	for (uint i = 0; i < markers.size();++i){
		auto &marker = markers[i];
		if (marker->getId() > maxId){
			maxId = marker->getId();
			maxIndex = i;
		}
		cv::Mat _rvec, _tvec, _rmat;
		cv::solvePnP(Marker::getObjectPoints(marker->getScale()), marker->points, mCamera, mDistCoef, _rvec, _tvec, false, cv::P3P);
		cv::solvePnP(Marker::getObjectPoints(marker->getScale()), marker->points, mCamera, mDistCoef, _rvec, _tvec, true);
		cv::Mat transform = cv::Mat::eye(4, 4, CV_32F);
		cv::Rodrigues(_rvec, _rmat);
		_rmat.copyTo(transform(cv::Rect(0, 0, 3, 3)));
		_tvec.copyTo(transform(cv::Rect(3, 0, 1, 3)));
		transforms.push_back(transform);
	}

	if (maxIndex != -1){
		auto &marker = markers[maxIndex];
		rmat = transforms[maxIndex](cv::Rect(0, 0, 3, 3)).clone();
		tvec = transforms[maxIndex](cv::Rect(3, 0, 1, 3)).clone();
		tvec *= 1.0f / 300;
		cv::Mat homography = cv::findHomography(marker->points, Marker::getPlannerPoints(marker->getScale()));
		for (uint i = 0; i < keypoints.size(); ++i){
			if (contains(marker->points, keypoints[i].pt)){
				mask[i] = true;
				cv::Mat vec = cv::Mat(3, 1, CV_64F);
				vec.at<double>(0) = keypoints[i].pt.x;
				vec.at<double>(1) = keypoints[i].pt.y;
				vec.at<double>(2) = 1;
				cv::Mat result = homography * vec;
				worldPos[i] = cv::Point3f(result.at<double>(0) / (300*result.at<double>(2)), result.at<double>(1) / ( 300*result.at<double>(2)), 0);
				//cout << "(" << worldPos[i].x << "," << worldPos[i].y << "," << worldPos[i].z << ")" << endl;
				ret += 1;
			}
		}
	}

	return ret;



}

void SimpleDetector::makeThreshold(const cv::Mat &in, cv::Mat &out){

	// No need for input image was already gray 
	//cv::Mat thres;
	//if (in.channels() == 3)
	//	cv::cvtColor(in, thres, cv::COLOR_BGR2GRAY);
	//else if (in.channels() == 4)
	//	cv::cvtColor(in, thres, cv::COLOR_BGRA2GRAY);

	cv::Canny(in, out, 10, 180);
}

inline double perimeter(std::vector<cv::Point> &vec){
	double sum = 0;
	for (uint i = 0; i < vec.size(); i++) {
		int i2 = (i + 1) % vec.size();
		sum += sqrt((vec[i].x - vec[i2].x) * (vec[i].x - vec[i2].x) + (vec[i].y - vec[i2].y) * (vec[i].y - vec[i2].y));
	}
	return sum;
}

void SimpleDetector::detectRectangles(cv::Mat &thresImage, std::vector<std::vector<cv::Point> > &rectangles){
	int maxSize = _maxSize * std::max(thresImage.cols, thresImage.rows) * 4;
	int minSize = std::min(float(_minSize_pix), _minSize* std::max(thresImage.cols, thresImage.rows) * 4);
	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > rects;
	std::vector<cv::Point> approx;
	cv::findContours(thresImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	for (uint i = 0; i<contours.size(); ++i){
		if (minSize<int(contours[i].size()) && int(contours[i].size())<maxSize){
			cv::approxPolyDP(contours[i], approx, double(contours[i].size()) * 0.05, true);

			if (approx.size()>4 && approx.size()<13){
				std::vector<cv::Point> approx2;
				for (uint j = 0; j<approx.size(); ++j){
					for (uint k = 0; k<approx2.size(); ++k){
						if ((approx[j].x - approx2[k].x)*(approx[j].x - approx2[k].x) + (approx[j].y - approx2[k].y)*(approx[j].y - approx2[k].y)<18){
							if (k == 0)
								std::reverse(approx2.begin(), approx2.end());
							goto tag;
						}
					}
					approx2.push_back(approx[j]);
				tag:
					//nvm
					;
				}
				approx = approx2;
			}

			if (approx.size() == 4 && cv::isContourConvex(approx)){
				float minDist = 1e10;
				for (int j = 0; j < 4; j++) {
					float d = std::sqrt((float)(approx[j].x - approx[(j + 1) % 4].x) * (approx[j].x - approx[(j + 1) % 4].x) +
						(approx[j].y - approx[(j + 1) % 4].y) * (approx[j].y - approx[(j + 1) % 4].y));
					if (d < minDist) minDist = d;
				}
				if (minDist>10){
					rects.push_back(approx);
				}
			}
		}
	}

	std::vector<std::pair<uint, uint>> tooNearRects;
	for (uint i = 0; i<rects.size(); ++i){
		for (uint j = i + 1; j<rects.size(); ++j){
			std::valarray<float> dists(4);
			for (int k = 0; k<4; ++k){
				dists[k] = std::sqrtf((rects[i][k].x - rects[j][k].x) * (rects[i][k].x - rects[j][k].x) +
					(rects[i][k].y - rects[j][k].y) * (rects[i][k].y - rects[j][k].y));
				if (dists[0] < 6 && dists[1] < 6 && dists[2] < 6 && dists[3] < 6) {
					tooNearRects.push_back(std::make_pair(i, j));
				}
			}
		}
	}

	std::valarray<bool> toRemove(false, rects.size());
	for (uint i = 0; i<tooNearRects.size(); ++i){
		if (toRemove[tooNearRects[i].first] || toRemove[tooNearRects[i].second])
			continue;
		if (perimeter(rects[tooNearRects[i].first])>perimeter(rects[tooNearRects[i].second]))
			toRemove[tooNearRects[i].second] = true;
		else
			toRemove[tooNearRects[i].first] = true;
	}

	rectangles.reserve(rects.size());
	for (uint i = 0; i<rects.size(); ++i){
		if (!toRemove[i]){
			rectangles.push_back(rects[i]);
		}
	}
}

cv::Mat SimpleDetector::warp(const cv::Mat &source, const std::vector<cv::Point> &rect, const cv::Size &size){
	cv::Point2f src[4], now[4];
	for (uint i = 0; i<4; ++i){
		src[i] = cv::Point2f(rect[i].x, rect[i].y);
	}
	now[0] = cv::Point2f(0, 0);
	now[1] = cv::Point2f(size.width - 1, 0);
	now[2] = cv::Point2f(size.width - 1, size.height - 1);
	now[3] = cv::Point2f(0, size.height - 1);
	cv::Mat tmp = cv::getPerspectiveTransform(src, now);
	cv::Mat ret;
	cv::warpPerspective(source, ret, tmp, size, cv::INTER_NEAREST);
	return ret;
}

void SimpleDetector::reOrder(std::vector<cv::Point> &points){
	float a1 = points[2].y - points[0].y, b1 = points[0].x - points[2].x, c1 = (points[2].x - points[0].x)*points[0].y - (points[2].y - points[0].y)*points[0].x;
	float a2 = points[3].y - points[1].y, b2 = points[1].x - points[3].x, c2 = (points[3].x - points[1].x)*points[1].y - (points[3].y - points[1].y)*points[1].x;
	cv::Point2f mid = cv::Point2f((b1*c2 - b2*c1) / (a1*b2 - a2*b1), (a2*c1 - a1*c2) / (a1*b2 - a2*b1));

	cv::Point2f v1 = cv::Point2f(points[0].x - mid.x, points[0].y - mid.y), v2 = cv::Point2f(points[1].x - mid.x, points[1].y - mid.y);
	float zvalue = v1.x*v2.y - v2.x*v1.y;
	if (zvalue > 0)
		return;
	else{
		std::swap(points[0], points[3]);
		std::swap(points[1], points[2]);
	}
}

bool SimpleDetector::contains(const std::vector<cv::Point2f> &points, const cv::Point2f &point){
	bool result = false;
	if (((points[0].y > point.y) != (points[3].y > point.y)) && (point.x < (points[3].x - points[0].x) * (point.y - points[0].y) / (points[3].y - points[0].y) + points[0].x))
		result = !result;
	if (((points[1].y > point.y) != (points[0].y > point.y)) && (point.x < (points[0].x - points[1].x) * (point.y - points[1].y) / (points[0].y - points[1].y) + points[1].x))
		result = !result;
	if (((points[2].y > point.y) != (points[1].y > point.y)) && (point.x < (points[1].x - points[2].x) * (point.y - points[2].y) / (points[1].y - points[2].y) + points[2].x))
		result = !result;
	if (((points[3].y > point.y) != (points[2].y > point.y)) && (point.x < (points[2].x - points[3].x) * (point.y - points[3].y) / (points[2].y - points[3].y) + points[3].x))
		result = !result;
	return result;
}
#include "PlainPaperDetector.h"
typedef unsigned int uint;

PlainPaperDetector::PlainPaperDetector(const string &strSettingsFile, int minFeatures) :MarkerDetector(strSettingsFile, minFeatures){
	this->initializable = false;
}

int PlainPaperDetector::detect(const cv::Mat &im, const std::vector<cv::KeyPoint> &keypoints, cv::Mat &rmat, cv::Mat &tvec, std::vector<bool> &mask, std::vector<cv::Point3f> &worldPos){
	std::vector<std::vector<cv::Point>> rectangles;
	int ret = 0;
	int maxIndex = -1;

	cv::Mat thres;
	makeThreshold(im, thres);
	detectRectangles(thres, rectangles);
	std::vector<bool> tmask = mask;
	for (int i = 0; i < rectangles.size();++i){
		std::vector<bool> tmask_ = mask;
		int result=adapt(keypoints, rectangles[i], tmask_);
		cv::Mat raw = warp(im, rectangles[i]);
		cv::Mat temp;
		if (raw.channels() == 3)
			cv::cvtColor(raw, temp, cv::COLOR_BGR2GRAY);
		else if (raw.channels() == 4)
			cv::cvtColor(raw, temp, cv::COLOR_BGRA2GRAY);
		cv::threshold(temp, temp, 100, 255, CV_THRESH_BINARY);
		double rate = ((double)cv::countNonZero(temp)) / (temp.cols*temp.rows);
		if (rate > 0.9&&result > ret){
			ret = result;
			tmask = tmask_;
			maxIndex = i;
		}
	}
	mask = tmask;

	if (maxIndex!=-1){
		std::vector<cv::Point3f> a4;
		cv::Mat _rvec, _tvec, _rmat;
		a4.push_back(cv::Point3f(-148.5, 105.0, 0));
		a4.push_back(cv::Point3f(-148.5, -105.0, 0));
		a4.push_back(cv::Point3f(148.5, -105.0, 0));
		a4.push_back(cv::Point3f(148.5, 105.0, 0));
		std::vector<cv::Point2f> temppoints;
		temppoints.push_back(cv::Point2f(rectangles[maxIndex][0]));
		temppoints.push_back(cv::Point2f(rectangles[maxIndex][1]));
		temppoints.push_back(cv::Point2f(rectangles[maxIndex][2]));
		temppoints.push_back(cv::Point2f(rectangles[maxIndex][3]));
		cv::solvePnP(a4, temppoints, mCamera, mDistCoef, _rvec, _tvec, false, cv::P3P);
		cv::solvePnP(a4, temppoints, mCamera, mDistCoef, _rvec, _tvec, true);
		cv::Rodrigues(_rvec, _rmat);
		rmat = _rmat;
		tvec = _tvec * (1.0f / 300);
		std::vector<cv::Point2f> a42d;
		a42d.push_back(cv::Point2f(-148.5, 105.0));
		a42d.push_back(cv::Point2f(-148.5, -105.0));
		a42d.push_back(cv::Point2f(148.5, -105.0));
		a42d.push_back(cv::Point2f(148.5, 105.0));
		cv::Mat homography = cv::findHomography(temppoints, a42d);
		for (uint i = 0; i < keypoints.size(); ++i){
			if (mask[i]){
				cv::Mat vec = cv::Mat(3, 1, CV_64F);
				vec.at<double>(0) = keypoints[i].pt.x;
				vec.at<double>(1) = keypoints[i].pt.y;
				vec.at<double>(2) = 1;
				cv::Mat result = homography * vec;
				worldPos[i] = cv::Point3f(result.at<double>(0) / (300 * result.at<double>(2)), result.at<double>(1) / (300 * result.at<double>(2)), 0);
			}
		}
	}

	return ret;
}

void PlainPaperDetector::makeThreshold(const cv::Mat &in, cv::Mat &out){
	cv::Mat thres;
	if (in.channels() == 3)
		cv::cvtColor(in, thres, cv::COLOR_BGR2GRAY);
	else if (in.channels() == 4)
		cv::cvtColor(in, thres, cv::COLOR_BGRA2GRAY);

	cv::Canny(thres, out, 10, 180);
}

inline double perimeter(std::vector<cv::Point> &vec){
	double sum = 0;
	for (uint i = 0; i < vec.size(); i++) {
		int i2 = (i + 1) % vec.size();
		sum += sqrt((vec[i].x - vec[i2].x) * (vec[i].x - vec[i2].x) + (vec[i].y - vec[i2].y) * (vec[i].y - vec[i2].y));
	}
	return sum;
}

void PlainPaperDetector::detectRectangles(cv::Mat &thresImage, std::vector<std::vector<cv::Point> > &rectangles){
	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > rects;
	std::vector<cv::Point> approx;
	cv::findContours(thresImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	for (uint i = 0; i<contours.size(); ++i){
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

cv::Mat PlainPaperDetector::warp(const cv::Mat &source, const std::vector<cv::Point> &rect, const cv::Size &size){
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

int PlainPaperDetector::adapt(const std::vector<cv::KeyPoint> &keypoints, std::vector<cv::Point> &rectangle, std::vector<bool> &mask){
	cv::Point2f r0(0, 0), r1(0, 0), r2(0, 0), r3(0, 0);
	bool b0 = false, b1 = false, b2 = false, b3 = false;
	auto &p0 = rectangle[0];
	auto &p1 = rectangle[1];
	auto &p2 = rectangle[2];
	auto &p3 = rectangle[3];
	int i0 = -1, i1 = -1, i2 = -1, i3 = -1;
	for (uint i = 0; i < keypoints.size(); ++i){
		if (abs(keypoints[i].pt.x - p0.x) + abs(keypoints[i].pt.y - p0.y)
			< abs(r0.x - p0.x) + abs(r0.y - p0.y) &&
			(keypoints[i].pt.x - p0.x)*(keypoints[i].pt.x - p0.x) + (keypoints[i].pt.y - p0.y)*(keypoints[i].pt.y - p0.y)<19){
			r0 = keypoints[i].pt;
			b0 = true;
			i0 = i;
		}
		if (abs(keypoints[i].pt.x - p1.x) + abs(keypoints[i].pt.y - p1.y)
			< abs(r1.x - p1.x) + abs(r1.y - p1.y) &&
			(keypoints[i].pt.x - p1.x)*(keypoints[i].pt.x - p1.x) + (keypoints[i].pt.y - p1.y)*(keypoints[i].pt.y - p1.y)<19){
			r1 = keypoints[i].pt;
			b1 = true;
			i1 = i;
		}
		if (abs(keypoints[i].pt.x - p2.x) + abs(keypoints[i].pt.y - p2.y)
			< abs(r2.x - p2.x) + abs(r2.y - p2.y) &&
			(keypoints[i].pt.x - p2.x)*(keypoints[i].pt.x - p2.x) + (keypoints[i].pt.y - p2.y)*(keypoints[i].pt.y - p2.y)<19){
			r2 = keypoints[i].pt;
			b2 = true;
			i2 = i;
		}
		if (abs(keypoints[i].pt.x - p3.x) + abs(keypoints[i].pt.y - p3.y)
			< abs(r3.x - p3.x) + abs(r3.y - p3.y) &&
			(keypoints[i].pt.x - p3.x)*(keypoints[i].pt.x - p3.x) + (keypoints[i].pt.y - p3.y)*(keypoints[i].pt.y - p3.y)<19){
			r3 = keypoints[i].pt;
			b3 = true;
			i3 = i;
		}
	}
	if (b0)
		p0 = r0;
	if (b1)
		p1 = r1;
	if (b2)
		p2 = r2;
	if (b3)
		p3 = r3;
	if (i0!=-1)
		mask[i0] = true;
	if (i1 != -1)
		mask[i1] = true;
	if (i2 != -1)
		mask[i2] = true;
	if (i3 != -1)
		mask[i3] = true;
	return b0 + b1 + b2 + b3;
}

void PlainPaperDetector::reOrder(std::vector<cv::Point> &points){
	float a1 = points[2].y - points[0].y, b1 = points[0].x - points[2].x, c1 = (points[2].x - points[0].x)*points[0].y - (points[2].y - points[0].y)*points[0].x;
	float a2 = points[3].y - points[1].y, b2 = points[1].x - points[3].x, c2 = (points[3].x - points[1].x)*points[1].y - (points[3].y - points[1].y)*points[1].x;
	cv::Point2f mid = cv::Point2f((b1*c2 - b2*c1) / (a1*b2 - a2*b1), (a2*c1 - a1*c2) / (a1*b2 - a2*b1));

	cv::Point2f v1 = cv::Point2f(points[0].x - mid.x, points[0].y - mid.y), v2 = cv::Point2f(points[1].x - mid.x, points[1].y - mid.y);
	float zvalue = v1.x*v2.y - v2.x*v1.y;
	if(zvalue<0){
		std::swap(points[0], points[3]);
		std::swap(points[1], points[2]);
	}

	auto temp = points;
	int max = 0;
	for (int i = 1; i<4; ++i){
		if (points[i].y>points[max].y)max = i;
	}
	if ((points[max] - points[(max + 1) % 4]).dot(points[max] - points[(max + 1) % 4])>(points[max] - points[(max - 1) % 4]).dot(points[max] - points[(max - 1) % 4]))
		max = (max - 1) % 4;
	points[0] = temp[max];
	points[1] = temp[(max + 1) % 4];
	points[2] = temp[(max + 2) % 4];
	points[3] = temp[(max + 3) % 4];
}
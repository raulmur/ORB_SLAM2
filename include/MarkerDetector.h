//nvm

#ifndef MARKERDETECTOR_H
#define MARKERDETECTOR_H

#include<opencv2/opencv.hpp>
#include"Frame.h"
#include<mutex>
#include<thread>
#include<chrono>
#include<memory>
#include "Utils.h"

#ifndef STDUSLEEP
#define STDUSLEEP
#define usleep(i) std::this_thread::sleep_for(std::chrono::microseconds(i))
#endif

#define MAX_DISTANCE_PIXEL 33

using namespace ORB_SLAM2;


class MarkerFrameData{
public:
	cv::Mat image;
	Frame frame;
	cv::Mat tvec;
	cv::Mat rmat;
	//how many keypoints are detected in the marker
	int detectedKeyPoints;
	//mask that marks up keypoints not in the marker
	std::vector<bool> mask;
	//world position of 2d keypoints
	std::vector<cv::Point3f> worldPos;

	//constructor
	inline MarkerFrameData(const cv::Mat &im, const Frame &fr) :frame(fr), detectedKeyPoints(0), image(im.clone()),
		mask(std::vector<bool>(fr.mvKeysUn.size(), false)), worldPos(std::vector<cv::Point3f>(fr.mvKeysUn.size(), cv::Point3f(-1, -1, -1))){

	}

	//copy constructor
	inline MarkerFrameData(const MarkerFrameData &another) :image(another.image.clone()), frame(another.frame),
		tvec(another.tvec.clone()), rmat(another.rmat.clone()), detectedKeyPoints(another.detectedKeyPoints),
		mask(another.mask),worldPos(another.worldPos){

	}

	//move constructor
	//the frame is currently just copied
	inline MarkerFrameData(MarkerFrameData &&another) :image(another.image), frame(another.frame), tvec(another.tvec),
		rmat(another.rmat), detectedKeyPoints(another.detectedKeyPoints), mask(std::move(another.mask)), worldPos(std::move(another.worldPos)){

	}

};

class MarkerDetector{
public:
	//the input image was already gratscale
	virtual int detect(const cv::Mat &im, const std::vector<cv::KeyPoint> &keypoints, cv::Mat &rmat, cv::Mat &tvec, std::vector<bool> &mask,std::vector<cv::Point3f> &worldPos);

	void addFrame(const cv::Mat &image, const Frame &frame);

	std::unique_ptr<MarkerFrameData> requestInit(uint32_t expectedRefineSamples);
	MarkerFrameData* requestEstimate(std::vector<uint32_t> &IDofKeyframes);

	inline void requestFinish(){
		finishRequested = true;
	}

	inline bool isFinished() const{
		return finished;
	}

	void run();
	void reset();

	MarkerDetector(const string &strSettingsFile, int minFeatures = 20);


	cv::Mat mDistCoef;
	cv::Mat mCamera;


protected:
	//marker estimated frames
	std::list<std::unique_ptr<MarkerFrameData>> assignedFrames;
	std::mutex assignedFramesMutex;

	//undetected frames
	std::list<std::unique_ptr<MarkerFrameData>> bufferedFrames;
	std::mutex bufferedFramesMutex;

	bool initializable;

//private functions
private:
	std::vector<const MarkerFrameData*> closestFrames(MarkerFrameData &data);
	int refine(MarkerFrameData &target, std::vector<const MarkerFrameData*> references);
	


private:
	int minInitFeatures;

	bool finishRequested;
	bool finished;

	int maxBufferFrames;
	int maxAssignedFrames;
	int defaultRefineFrames;
};











#endif
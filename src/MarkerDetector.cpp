#include"MarkerDetector.h"
#include"ORBmatcher.h"
typedef unsigned int uint;

void MarkerDetector::addFrame(const cv::Mat &image, const Frame &frame){
	std::unique_lock<std::mutex> lock(bufferedFramesMutex);
	bufferedFrames.push_back(std::make_unique<MarkerFrameData>(image, frame));
}

int MarkerDetector::detect(const cv::Mat &im, const std::vector<cv::KeyPoint> &keypoints, cv::Mat &rmat, cv::Mat &tvec, std::vector<bool> &mask, std::vector<cv::Point3f> &worldPos){
	std::cout << "Error! call without override" << std::endl;
	return -1;
}

MarkerDetector::MarkerDetector(const string &strSettingsFile, int minFeatures):minInitFeatures(minFeatures),
	finishRequested(false), finished(false), initializable(true){
	cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
	float fx = fSettings["Camera.fx"];
	float fy = fSettings["Camera.fy"];
	float cx = fSettings["Camera.cx"];
	float cy = fSettings["Camera.cy"];

	cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
	K.at<float>(0, 0) = fx;
	K.at<float>(1, 1) = fy;
	K.at<float>(0, 2) = cx;
	K.at<float>(1, 2) = cy;
	K.copyTo(mCamera);

	cv::Mat DistCoef(4, 1, CV_32F);
	DistCoef.at<float>(0) = fSettings["Camera.k1"];
	DistCoef.at<float>(1) = fSettings["Camera.k2"];
	DistCoef.at<float>(2) = fSettings["Camera.p1"];
	DistCoef.at<float>(3) = fSettings["Camera.p2"];
	const float k3 = fSettings["Camera.k3"];
	if (k3 != 0)
	{
		DistCoef.resize(5);
		DistCoef.at<float>(4) = k3;
	}
	DistCoef.copyTo(mDistCoef);

	if (!fSettings["MaxBufferedFrames"].empty()){
		maxBufferFrames = fSettings["MaxBufferedFrames"];
		if (maxBufferFrames < 5)
			maxBufferFrames = 5;
	}
	else{
		maxBufferFrames = 10;
	}

	if (!fSettings["MaxAssignedFrames"].empty()){
		maxAssignedFrames = fSettings["MaxAssignedFrames"];
		if (maxAssignedFrames < 20)
			maxAssignedFrames = 20;
	}
	else{
		maxAssignedFrames = 30;
	}

	if (!fSettings["DefaultRefineFrames"].empty()){
		defaultRefineFrames = fSettings["DefaultRefineFrames"];
		if (defaultRefineFrames > 5)
			defaultRefineFrames = 5;
	}
	else{
		defaultRefineFrames = -1;
	}
}

void MarkerDetector::run(){
	while (!finishRequested){
		std::unique_ptr<MarkerFrameData> markerFrameData;
		int tempBufferSize;
		{
			std::unique_lock<std::mutex> lock(bufferedFramesMutex);
			tempBufferSize = bufferedFrames.size();
		

			if (tempBufferSize == 0){
				lock.unlock();
				usleep(2000);
				continue;
			}

			if (tempBufferSize > maxBufferFrames){
				for (uint i = 0; i < tempBufferSize - maxBufferFrames; ++i){
					bufferedFrames.pop_front();
				}
			}
			markerFrameData = std::move(bufferedFrames.back());
			bufferedFrames.pop_back();
		}
		markerFrameData->detectedKeyPoints = detect(markerFrameData->image, markerFrameData->frame.mvKeysUn, markerFrameData->rmat, markerFrameData->tvec, markerFrameData->mask, markerFrameData->worldPos);
		if (markerFrameData->detectedKeyPoints >= minInitFeatures){
			std::unique_lock<std::mutex> lock(assignedFramesMutex);
			assignedFrames.push_back(std::move(markerFrameData));
			while (assignedFrames.size() > maxAssignedFrames){
				assignedFrames.pop_front();
			}
		}
	}
	finished = true;
}

std::vector<const MarkerFrameData*> MarkerDetector::closestFrames(MarkerFrameData &data){
	std::vector<const MarkerFrameData*> vec;
	cv::Mat mat0 = cv::Mat::eye(4, 4, CV_32F);
	data.rmat.copyTo(mat0(cv::Rect(0, 0, 3, 3)));
	data.tvec.copyTo(mat0(cv::Rect(3, 0, 1, 3)));
	std::vector<std::pair<float, MarkerFrameData*>> dist_ptr;
	for (auto i = assignedFrames.begin(); i != assignedFrames.end();++i){
		if (i->get() == &data)
			continue;
		cv::Mat mat = cv::Mat::eye(4, 4, CV_32F);
		i->get()->rmat.copyTo(mat(cv::Rect(0, 0, 3, 3)));
		i->get()->tvec.copyTo(mat(cv::Rect(3, 0, 1, 3)));
		cv::Mat temp = (mat.inv() - mat0.inv()).col(3);
		dist_ptr.push_back(std::make_pair(sqrtf(temp.at<float>(0)*temp.at<float>(0) + temp.at<float>(1)*temp.at<float>(1) + temp.at<float>(2)*temp.at<float>(2)), i->get()));
	}
	std::sort(dist_ptr.begin(), dist_ptr.end());

	for (uint i = 0; i < dist_ptr.size(); ++i){
		vec.push_back(dist_ptr[i].second);
	}
	return vec;
}

int MarkerDetector::refine(MarkerFrameData &target, std::vector<const MarkerFrameData*> references){
	if (references.size() == 0)
		return target.detectedKeyPoints;
	std::vector<std::vector<std::pair<int, int>>> ref_index(target.image.cols, std::vector<std::pair<int, int>>(target.image.rows, make_pair(-1, -1)));
	cv::Mat h0 = Utils::findHomography(mCamera, target.rmat, target.tvec);

	//stores index of each descriptor into a map
	for (uint i = 0; i < references.size(); ++i){
		cv::Mat hi = Utils::findHomography(mCamera, references[i]->rmat, references[i]->tvec).inv();
		for (uint j = 0; j < references[i]->frame.mvKeysUn.size(); ++j){
			if (!references[i]->mask[j])
				continue;
			if (references[i]->frame.mvKeysUn[j].octave>0)
				continue;
			cv::Mat temp = h0*hi*(cv::Mat_<float>(3, 1) << references[i]->frame.mvKeysUn[j].pt.x, references[i]->frame.mvKeysUn[j].pt.y, 1);
			if (temp.at<float>(0) < 0 || temp.at<float>(0) > target.image.cols || temp.at<float>(1) < 0 || temp.at<float>(0) > target.image.rows)
				continue;
			ref_index[temp.at<float>(0)][temp.at<float>(1)] = std::make_pair(i,j);
		}
	}

	//refine the mask
	int ret = 0;
	for (uint i = 0; i < target.frame.mvKeysUn.size(); ++i){
		if (!target.mask[i])
			continue;
		if (target.frame.mvKeysUn[i].octave>0)
			continue;
		int minDistance = 9999;
		std::pair<int, int> minIndex = std::make_pair(-1, -1);
		for (uint j = target.frame.mvKeysUn[i].pt.x - 2; j <= target.frame.mvKeysUn[i].pt.x + 2; ++j){
			for (uint k = target.frame.mvKeysUn[i].pt.y - 2; k <= target.frame.mvKeysUn[i].pt.y + 2; ++k){
				if (ref_index[j][k].first == -1)
					continue;
				int distance = ORBmatcher::DescriptorDistance(target.frame.mDescriptors.row(i), references[ref_index[j][k].first]->frame.mDescriptors.row(ref_index[j][k].second));
				if (distance < minDistance){
					minDistance = distance;
					minIndex = std::make_pair(j, k);
				}
			}
		}
		if (minIndex.first != -1 && minDistance < ORBmatcher::TH_LOW){
			ret += 1;
			ref_index[minIndex.first][minIndex.second] = make_pair(-1, -1);
		}
		else{
			target.mask[i] = false;
		}
	}
	target.detectedKeyPoints = ret;
	return ret;
}

std::unique_ptr<MarkerFrameData> MarkerDetector::requestInit(uint expectedRefineSamples){
	if (defaultRefineFrames >= 0)
		expectedRefineSamples = (uint)defaultRefineFrames;
	std::unique_lock<std::mutex> lock(assignedFramesMutex);
	if (assignedFrames.size() < 1)
		return std::unique_ptr<MarkerFrameData>();
	if (!initializable)
		return std::unique_ptr<MarkerFrameData>();
	std::unique_ptr<MarkerFrameData> unique = std::make_unique<MarkerFrameData>(*(assignedFrames.back().get()));
	auto temp = closestFrames(*unique);
	if (temp.size() > expectedRefineSamples)
		temp.resize(expectedRefineSamples);
	if (refine(*unique, temp)>minInitFeatures){
		return unique;
	}
	else{
		return std::unique_ptr<MarkerFrameData>();
	}
}

MarkerFrameData* MarkerDetector::requestEstimate(std::vector<uint32_t> &IDofKeyframes){
	MarkerFrameData *ret = static_cast<MarkerFrameData*>(NULL);
	std::sort(IDofKeyframes.begin(), IDofKeyframes.end());
	std::unique_lock<std::mutex> lock(assignedFramesMutex);
	auto li = assignedFrames.begin();
	auto vi = IDofKeyframes.begin();
	while (li != assignedFrames.end() && vi != IDofKeyframes.end()){
		if (li->get()->frame.mnId == *vi){
			ret = li->get();
			break;
		}
		else if (li->get()->frame.mnId > *vi){
			vi++;
		}
		else{
			li++;
		}
	}
	return ret;
}

void MarkerDetector::reset(){
	std::unique_lock<std::mutex> lock(bufferedFramesMutex);
	std::unique_lock<std::mutex> lock2(assignedFramesMutex);
	bufferedFrames.clear();
	assignedFrames.clear();
	return;
}
#include "ObjectTracking.h"

namespace ORB_SLAM2
{

ObjectTracking::ObjectTracking(const std::string &modelPath, const bool useGPU, const cv::Size &size, const float confidenceThreshold, const float iouThreshold, const float maxCosineDistance, const int nnBudget)
{
    mbIsGPU = useGPU;
    mSize = size;
    mConfidenceThreshold = confidenceThreshold;
    mIOUThreshold = iouThreshold;
    mMaxCosineDistance = maxCosineDistance;
    mNNBudget = nnBudget;

    mpDetector = new YOLO(modelPath, mbIsGPU, mSize);
    mpByteTracker = new ByteTrack(mMaxCosineDistance, mNNBudget);
}


void ObjectTracking::Run()
{
    track(mImage);
}

void ObjectTracking::track(cv::Mat &image)
{
    // use YOLO detector
    mDetections = mpDetector->detect(image, mConfidenceThreshold, mIOUThreshold);
    // use ByteTrack tracker
    mTrackResults = mpByteTracker->update(mDetections);
}
}

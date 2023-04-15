#include "ObjectTracking.h"

namespace ORB_SLAM2
{

ObjectTracking::ObjectTracking(const std::string &modelPath, const bool isGPU, const cv::Size& size, const float confidence_threshold, const float iou_threshold, const float max_cosine_distance, const int nn_budget)
{
    mpDetector = new YOLO(modelPath, isGPU, size);
    mpByteTracker = new ByteTrack(max_cosine_distance, nn_budget);

    mbIsGPU = isGPU;
    mSize = size;
    mConfidenceThreshold = confidence_threshold;
    mIOUThreshold = iou_threshold;
    mMaxCosineDistance = max_cosine_distance;
    mNNBudget = nn_budget;

}

ObjectTracking::~ObjectTracking()
{
    delete mpDetector;
    delete mpByteTracker;
}

void ObjectTracking::Run()
{
    std::vector<Detection> results = mpDetector->detect(mImage, mConfidenceThreshold, mIOUThreshold);
}

}
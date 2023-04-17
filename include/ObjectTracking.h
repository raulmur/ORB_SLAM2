#ifndef OBJECTTRACKING_H
#define OBJECTTRACKING_H

#include <opencv2/opencv.hpp>

#include "ByteTrack/detector.h"
#include "ByteTrack/BYTETrack.h"

namespace ORB_SLAM2
{

class ObjectTracking
{
    public:
        ObjectTracking(const std::string &modelPath, const bool useGPU, const cv::Size &size, const float confidenceThreshold, const float iouThreshold, const float maxCosineDistance, const int nnBudget);

        ~ObjectTracking();

        void Run();

        void track(cv::Mat &image);
        
        std::vector<Detection> GetDetections() { return mDetections; }
        std::vector<STrack> GetTrackResults() { return mTrackResults; }

    public:
        cv::Mat mImage;

    public:
        YOLO *mpDetector;
        BYTETrack *mpByteTracker;

        bool mbIsGPU;
        cv::Size mSize;
        float mConfidenceThreshold;
        float mIOUThreshold;
        float mMaxCosineDistance;
        int mNNBudget;

        std::vector<Detection> mDetections;
        std::vector<STrack> mTrackResults;





};

}

#endif // OBJECTTRACKING_H
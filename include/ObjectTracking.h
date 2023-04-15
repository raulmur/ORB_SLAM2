#include <opencv2/opencv.hpp>

#include "ByteTrack/detector.h"
#include "ByteTrack/ByteTrack.h"

namespace ORB_SLAM2
{

class ObjectTracking
{
    public:
        ObjectTracking(const std::string &modelPath, const bool isGPU, const cv::Size &size, const float confidence_threshold, const float iou_threshold, const float max_cosine_distance, const int nn_budget);

        ~ObjectTracking();

        void Run();
        

    public:
        cv::Mat mImage;

    protected:
        YOLO* mpDetector;
        ByteTrack* mpByteTracker;

        bool mbIsGPU;
        cv::Size mSize;
        float mConfidenceThreshold;
        float mIOUThreshold;
        float mMaxCosineDistance;
        int mNNBudget;





};

}
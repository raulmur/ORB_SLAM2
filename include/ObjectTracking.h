#include <iostream>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{

class Tracking;

class ObjectTracking
{
    public:
        ObjectTracking(const std::string &modelPath);
        ~ObjectTracking();

        void TrackObject(cv::Mat im, cv::Mat imDepth);


    private:

    

};





}
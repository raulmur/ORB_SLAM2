#pragma once
#ifndef ORBSLAMPY_ORBSLAMMER_H
#define ORBSLAMPY_ORBSLAMMER_H

// SYSTEM INCLUDES
#include <string>
#include <opencv2/core/core.hpp>
#include <ORB_SLAM2/System.h> // ORB_SLAM2 header

// C++ PROJECT INCLUDES
#include "_orbslampy/pyvector.h"

enum SensorType
{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
};

class OrbSlammer
{
public:
    OrbSlammer(const std::string& vocabFilePath, const std::string& settingsFilePath,
               SensorType sensor, bool useViewer=false);

    virtual ~OrbSlammer();

    cv::Mat TrackStereo(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timeStamp);

    cv::Mat TrackRGBD(const cv::Mat& im, const cv::Mat& depthMap, const double& timeStamp);

    cv::Mat TrackMonocular(const cv::Mat& im, const double& timeStamp);

    void ActivateLocalizationMode();

    void DeactivateLocalizationMode();

    bool MapChanged();

    void Reset();

    void Shutdown();

    void SaveTrajectoryTUM(const std::string& filePath);

    void SaveKeyFrameTrajectoryTUM(const std::string& filePath);

    void SaveTrajectoryKITTI(const std::string& filePath);

    int GetTrackingState();

    std::vector<cv::Mat> GetMostRecentPointCloud();

    std::vector<cv::Mat> GetWorldPointCloud();

private:

    bool                _is_running;
    ORB_SLAM2::System   _slam_system;
    unsigned int        _num_tracked_points;

};

#endif // end of ORBSLAMPY_ORBSLAMMER_H

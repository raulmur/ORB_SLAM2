// SYSTEM INCLUDES


// C++ PROJECT INCLUDES
#include "_orbslampy/support_functions.h"
#include "_orbslampy/orbslammer.h"

OrbSlammer::OrbSlammer(const std::string& vocabFilePath, const std::string& settingsFilePath,
                       SensorType sensor, bool useViewer) : _is_running(true),
    _slam_system(vocabFilePath, settingsFilePath, get_raw_sensor_type(sensor), useViewer),
    _num_tracked_points(0)
{
}

OrbSlammer::~OrbSlammer()
{
    if(this->_is_running)
    {
        this->_is_running = false;
        this->Shutdown();
    }
}

cv::Mat OrbSlammer::TrackStereo(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timeStamp)
{
    return this->_slam_system.TrackStereo(imLeft, imRight, timeStamp);
}

cv::Mat OrbSlammer::TrackRGBD(const cv::Mat& im, const cv::Mat& depthMap, const double& timeStamp)
{
    return this->_slam_system.TrackRGBD(im, depthMap, timeStamp);
}

cv::Mat OrbSlammer::TrackMonocular(const cv::Mat& im, const double& timeStamp)
{
    return this->_slam_system.TrackMonocular(im, timeStamp);
}

void OrbSlammer::ActivateLocalizationMode()
{
    this->_slam_system.ActivateLocalizationMode();
}

void OrbSlammer::DeactivateLocalizationMode()
{
    this->_slam_system.DeactivateLocalizationMode();
}

bool OrbSlammer::MapChanged()
{
    return this->_slam_system.MapChanged();
}

void OrbSlammer::Reset()
{
    this->_slam_system.Reset();
}

void OrbSlammer::Shutdown()
{
    this->_slam_system.Shutdown();
}

void OrbSlammer::SaveTrajectoryTUM(const std::string& filePath)
{
    this->_slam_system.SaveTrajectoryTUM(filePath);
}

void OrbSlammer::SaveKeyFrameTrajectoryTUM(const std::string& filePath)
{
    this->_slam_system.SaveKeyFrameTrajectoryTUM(filePath);
}

void OrbSlammer::SaveTrajectoryKITTI(const std::string& filePath)
{
    this->_slam_system.SaveTrajectoryKITTI(filePath);
}

int OrbSlammer::GetTrackingState()
{
    return this->_slam_system.GetTrackingState();
}

std::vector<cv::Mat> OrbSlammer::GetMostRecentPointCloud()
{
    std::vector<cv::Mat> point_cloud;
    std::vector<ORB_SLAM2::MapPoint*> all_map_points = this->_slam_system.GetTrackedMapPoints();
    unsigned int current_num_tracked_points = all_map_points.size();
    for(; this->_num_tracked_points < current_num_tracked_points; ++this->_num_tracked_points)
    {
        point_cloud.push_back(all_map_points[this->_num_tracked_points]->GetWorldPos());
    }
    return point_cloud;
}

std::vector<cv::Mat> OrbSlammer::GetWorldPointCloud()
{
    std::vector<cv::Mat> point_cloud;
    std::vector<ORB_SLAM2::MapPoint*> all_map_points;
    unsigned int all_map_points_size = all_map_points.size();
    for(unsigned int i = 0; i < all_map_points_size; ++i)
    {
        point_cloud.push_back(all_map_points[i]->GetWorldPos());
    }
    return point_cloud;
}

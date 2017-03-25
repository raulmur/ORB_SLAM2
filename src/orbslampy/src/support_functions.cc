// SYSTEM INCLUDES


// C++ PROJECT INCLUDES
#include "_orbslampy/support_functions.h"


const ORB_SLAM2::System::eSensor get_raw_sensor_type(const SensorType sType)
{
    if(sType == SensorType::MONOCULAR)
    {
        return ORB_SLAM2::System::eSensor::MONOCULAR;
    }
    else if(sType == SensorType::STEREO)
    {
        return ORB_SLAM2::System::eSensor::STEREO;
    }
    else
    {
        return ORB_SLAM2::System::eSensor::RGBD;
    }
}

#pragma once
#ifndef ORBSLAMPY_SUPPORT_FUNCTIONS_H
#define ORBSLAMPY_SUPPORT_FUNCTIONS_H

// SYSTEM INCLUDES
#include <ORB_SLAM2/System.h> // ORB_SLAM2 header

// C++ PROJECT INCLUDES
#include "_orbslampy/orbslammer.h"

const ORB_SLAM2::System::eSensor get_raw_sensor_type(const SensorType sType);

#endif // end of ORBSLAMPY_SUPPORT_FUNCTIONS_H

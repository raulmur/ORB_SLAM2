/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <unistd.h>

// ZED include
#include <sl/Camera.hpp>
#include "utils.hpp"
#include <sys/resource.h>
#include <stdio.h>

using namespace std;
using namespace sl;

// #define SVO_PATH "/home/pengfei/Documents/wtc_square_loop.svo"
// #define SVO_PATH "/media/pengfei/Data/track_1.svo"
#define SVO_PATH "/media/pengfei/Data/Sample/hello.svo"

void runORBSLAMTracking (ORB_SLAM2::System *ptrToORBSLAMSystem, sl::Mat zedLeftView, sl::Mat zedRightView, int timeCounter);

int main(int argc, char **argv)
{
    // ================== Set stack size =============//
    // Fix map saving/loading bug (/ORB_SLAM2/pull/381)
    const rlim_t kStackSize = 16L * 2014L * 1024L;
    struct rlimit rl;
    int result;

    result = getrlimit(RLIMIT_STACK, &rl);
    if (result == 0)
    {
        if (rl.rlim_cur < kStackSize)
        {
            rl.rlim_cur = kStackSize;
            result = setrlimit(RLIMIT_STACK, &rl);
            if (result != 0)
            {
                fprintf(stderr, "setrlimit returned result = %d\n", result);
            }
        }
    }

    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System* slamPtr = new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::STEREO,true, false);

    Camera zed;
    InitParameters initParameters;
    initParameters.camera_resolution = RESOLUTION_HD720;
    initParameters.svo_input_filename.set(SVO_PATH);

    // Open the ZED
    ERROR_CODE err = zed.open(initParameters);
    if (err != SUCCESS) {
        cout << toString(err) << endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    int width = zed.getResolution().width;
    int height = zed.getResolution().height;
    cout << "Width: " << width << endl;
    cout << "Height: " << height << endl;


    sl::Mat leftView;
    sl::Mat rightView;

    const int nb_frames = zed.getSVONumberOfFrames();
    const int startIndex = 0;

    for(int frame_index = startIndex; frame_index < nb_frames; frame_index++)
    {
        if(zed.grab() == SUCCESS & (frame_index % 2 == 0))
        {
            zed.retrieveImage(leftView, VIEW_LEFT);
            zed.retrieveImage(rightView, VIEW_RIGHT);
            auto start = chrono::system_clock::now();
            runORBSLAMTracking(slamPtr, leftView, rightView, frame_index);
            auto end = chrono::system_clock::now();
            auto duration = end - start;
            long millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
            cout << "Time: " << millis << " Frame: " << frame_index << endl;
            cout << "Time stamp of image: " << zed.getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE) << endl;
            cout << "Now: " << chrono::system_clock::now().time_since_epoch() / chrono::milliseconds(1) << endl;
        }
    }
    zed.close();

    std::this_thread::sleep_for(std::chrono::microseconds(5000));
    // Stop all threads
    slamPtr->Shutdown();
   // Save camera trajectory
    slamPtr->SaveTrajectoryKITTI("CameraTrajectory_KITTI_06.txt");

    return 0;
}

void runORBSLAMTracking (ORB_SLAM2::System *ptrToORBSLAMSystem, sl::Mat zedLeftView, sl::Mat zedRightView, int timeCounter)
{
    //Note that sl::Mat here in a 8UC4 (RGBA) format.
    cv::Mat leftViewCV, rightViewCV, leftViewCVGray, rightViewCVGray;
    //Note that cv::Mat here in a 8UC4 (BGRA) format.
    leftViewCV = slMat2cvMat(zedLeftView);
    rightViewCV = slMat2cvMat(zedRightView);

    //convert BGRA -> BGR
    cv::cvtColor(leftViewCV, leftViewCV, CV_BGRA2BGR);
    cv::cvtColor(rightViewCV, rightViewCV, CV_BGRA2BGR);
    //convert BGR -> RGB
    cv::cvtColor(leftViewCV, leftViewCV, CV_BGR2RGB);
    cv::cvtColor(rightViewCV, rightViewCV, CV_BGR2RGB);

    cv::Mat Tcw;

    Tcw = ptrToORBSLAMSystem->TrackStereo(leftViewCV, rightViewCV, timeCounter);
}

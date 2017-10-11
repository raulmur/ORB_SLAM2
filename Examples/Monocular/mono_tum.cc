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
#include<chrono>
#include <ctime>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;


int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images

    int nImages = 0;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat im;
    cv::VideoCapture cap;
    cap.open(0);
    int key = 0;
    //for(int ni=0; ni<nImages; ni++)
    double starttime = static_cast<double>(std::time(nullptr));
    double nowtime = static_cast<double>(std::time(nullptr));
    while(key != 27)
    {
        // Read image from file
        cap >> im;
        cv::cvtColor(im, im, CV_RGB2GRAY);

        if(im.empty())
        {
            cerr << endl << "Failed to load image" << endl;
            return 1;
        }

        nowtime = static_cast<double>(std::time(nullptr));
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,nowtime-starttime);

        nImages++;
        
        key = cv::waitKey(1);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    float totaltime = nowtime-starttime;

    cout << "-------" << endl << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

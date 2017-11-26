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

#pragma warning( disable : 4996 )

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include"SimpleDetector.h"
#include"PlainPaperDetector.h"

#include<opencv2/core/core.hpp>

#include<System.h>
#define usleep_win(i) Sleep(i/1000)

using namespace std;

int main(int argc, char **argv)
{

	if (argc != 3)
	{
		cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings" << endl;
		return 1;
	}

	cv::VideoCapture* cap = new cv::VideoCapture(0);
	if (!cap->isOpened()){
		cerr << "Unable to get the camera" << endl;
		exit(-1);
	}

	//std::ofstream out("out.txt");
	//std::streambuf *coutbuf = std::cout.rdbuf();
	//std::cout.rdbuf(out.rdbuf()); 


	cap->set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap->set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    

	SimpleDetector *detector = new SimpleDetector(argv[2], 40);
	//PlainPaperDetector *detector = new PlainPaperDetector(argv[2]);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, detector);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << 10000 << endl << endl;

	//timestamp
	std::chrono::time_point<std::chrono::system_clock> t1, t2,t3;
	t1 = std::chrono::system_clock::now();

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<10000; ni++)
    {
        // Read image from file
		if (!cap->grab())
			exit(-1);
		cap->retrieve(im);

        if(im.empty())
        {
            cerr << endl << "Failed to load image!"<< endl;
            return 1;
        }

		t2 = std::chrono::system_clock::now();
		std::chrono::duration<double> timestamp = t2 - t1;
		t1 = t2;

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,timestamp.count());
		t3 = std::chrono::system_clock::now();
		std::chrono::duration<double, std::milli> timeesplashed = t3 - t2;


        // Wait to load the next frame
		if (timeesplashed.count()<33)
			usleep_win((33 - timeesplashed.count())*1e3);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
	cap->release();
	delete cap;

	delete detector;

    return 0;
}
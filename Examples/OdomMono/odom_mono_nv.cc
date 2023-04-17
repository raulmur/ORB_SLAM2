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

#include<System.h>

using namespace std;
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./odom_mono_nv path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    string orbBinFile = argv[1];
    string settingFile = argv[2];
    string dataPath = argv[3];

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    ORB_SLAM2::System SLAM(orbBinFile, settingFile, ORB_SLAM2::System::MONOCULAR,true);

    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);

    // Number of images
    int nImages = (int)fsSettings["Images.Number"];

    // Between-frame duration
    double T = (double)fsSettings["Images.FrameDuration"];

    // Camera parameters for pre-process of undistortion
    cv::Mat preK, preD;
    fsSettings["Camera.Pre.K"] >> preK;
    fsSettings["Camera.Pre.D"] >> preD;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    string fullOdomName = dataPath + "/odo_raw.txt";
    ifstream odomFile(fullOdomName);
    float x, y, theta;
    string odomLine;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    // Main loop
    cv::Mat im, imraw;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        string fullImgName = dataPath + "/image/" + to_string(ni) + ".bmp";
        imraw = cv::imread(fullImgName, CV_LOAD_IMAGE_GRAYSCALE);
        cv::undistort(imraw, im, preK, preD);
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << fullImgName << endl;
            return 1;
        }

        // Read odometry data
        getline(odomFile, odomLine);
        istringstream iss(odomLine);
        iss >> x >> y >> theta;
        Se2 odom(x, y, theta);

        double tframe = 0;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        SLAM.TrackOdomMono(im, odom, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryVN("KeyFrameTrajectory.txt");

    return 0;
}
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}

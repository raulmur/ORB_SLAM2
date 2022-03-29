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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"

#include <iostream>
#include <dirent.h>
#include <string.h>
#include <string>
using namespace std;

// 读取图像路径和时间戳
void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl
             << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    // Main loop
    cv::Mat im;
    // cv::Mat before_im;
    
    // 倒序读取
    // sort(vTimestamps.begin(), vTimestamps.end(), [](double a, double b)
    //      { return a > b; });
    
    for (int ni = 0; ni < nImages; ni++)
    {
        // Read image from file
        // cout << "vstrImageFilenames[ni]: " << vstrImageFilenames[ni] << endl;
        im = cv::imread(vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
        // before_im = cv::imread(vstrImageFilenames[ni], CV_LOAD_IMAGE_GRAYSCALE);
        // cv::equalizeHist(before_im, im); // 直方图均衡化，效果较差

        double tframe = vTimestamps[ni];
        // cout << "vTimestamps[ni]" << vTimestamps[ni] << endl;

        if (im.empty())
        {
            cerr << endl
                 << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        cout << "Frames ID: " << ni << endl;
        SLAM.TrackMonocular(im, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // // Wait to load the next frame
        // double T = 0;
        // if (ni < nImages - 1)
        //     T = vTimestamps[ni + 1] - tframe;
        // else if (ni > 0)
        //     T = tframe - vTimestamps[ni - 1];

        // if (ttrack < T)
        //     usleep((T - ttrack) * 1e6);

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = tframe - vTimestamps[ni + 1];
        else if (ni > 0)
            T = vTimestamps[ni - 1] - tframe;

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);

    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    // read timestamps
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    // read images
    string strPrefixLeft = strPathToSequence + "/image_0/";
    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    //    for(int i=0; i<nTimes; i++)
    //    {
    //        stringstream ss;
    //        ss << setfill('0') << setw(6) << i;
    //        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    //    }

    char *dirname = const_cast<char *>(strPrefixLeft.c_str());
    DIR *d = opendir(dirname);
    if (d == NULL)
    {
        printf("d == NULL");
    }
    struct dirent *entry;
    int min = 999999;
    while ((entry = readdir(d)) != NULL)
    {
        string s(entry->d_name, strlen(entry->d_name));
        if (s.length() >= 6)
        {
            string ss = s.substr(0, 6);
            int i = stoi(ss);
            if (min > i)
                min = i;
        }
    }
    closedir(d);
    cout << "min:" << min << endl;
    for (int i = 0; i < nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i + min;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }

    // 倒序读取
    // for (int i = 0; i < nTimes; i++)
    // {
    //     stringstream ss;
    //     ss << setfill('0') << setw(6) << i + min;
    //     vstrImageFilenames[nTimes - i - 1] = strPrefixLeft + ss.str() + ".png";
    // }
}

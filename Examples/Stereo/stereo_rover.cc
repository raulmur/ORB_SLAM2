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
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

void LoadRoverImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
 


    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadRoverImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Choose if you need Pure Localization
    // char IsPureLocalization;
    // cout << "Do you want to run pure localization? (Y/N)" << endl;  
    // cin >> IsPureLocalization;  
    // if(IsPureLocalization == 'Y' || IsPureLocalization == 'y'){
    //     SLAM.ActivateLocalizationMode();
    // }
    // else{
    //     SLAM.DeactivateLocalizationMode();
    // }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    bool man_insert;
    char type_in;
    cout << "Do you want to manually feed in the images? (y/n)" << endl;
    cin >> type_in;
    if(type_in == 'Y' || type_in == 'y'){  
        man_insert = true;
    }
    else{
        man_insert = false;
    }

    for(int ni=0; ni<nImages; ni++)
    {   
        if(man_insert){
            cout << "Please type something in so that we can load another image." << endl;
            cin >> type_in;
        }
        

        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];
        // cout << "stereo_zed.cc :: Now Time gap =" << T << endl;

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
        // usleep(0.03*1e6);
    }

    cout << "Read all images. Now trying to shut down." << endl << endl;
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    cout << "Calculating tracking time......" << endl << endl;
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "---------------------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    // Save customized Map
    char IsSaveMap;  
    cout << "Do you want to save the map?(y/n)" << endl;  
    cin >> IsSaveMap;  
    if(IsSaveMap == 'Y' || IsSaveMap == 'y')  
        SLAM.SaveMap("MapPointandKeyFrame.bin");

    return 0;
}

void LoadRoverImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimesLeft;
    ifstream fTimesRight;
    vector<double> vTimestampsLeft;
    vector<double> vTimestampsRight;
    string strPathTimeFileLeft = strPathToSequence + "/left.txt";
    string strPathTimeFileRight = strPathToSequence + "/right.txt";
    fTimesLeft.open(strPathTimeFileLeft.c_str());
    fTimesRight.open(strPathTimeFileRight.c_str());
    // Read in all Timestamps for left images
    while(!fTimesLeft.eof()){
        string s_l;
        getline(fTimesLeft,s_l);
        if(!s_l.empty()){
            stringstream ss_l;
            double t_l;
            ss_l << s_l;
            ss_l >> t_l;
            vTimestampsLeft.push_back(t_l);
        }
    }
    
    // Read in all Timestamps for right images
    while(!fTimesRight.eof()){
        string s_r;
        getline(fTimesRight,s_r);
        if(!s_r.empty()){
            stringstream ss_r;
            double t_r;
            ss_r << s_r;
            ss_r >> t_r;
            vTimestampsRight.push_back(t_r);
        }
    }
    
    const int nTimesLeft = vTimestampsLeft.size();
    const int nTimesright = vTimestampsRight.size();
    int i=0;
    int j=0;
    int nwrongmatch = 0;
    // Pair all the images with timestamp gap < 0.03s
    while(i<nTimesLeft && j<nTimesright){
        if(vTimestampsLeft[i] == vTimestampsRight[j]){
            vTimestamps.push_back(vTimestampsLeft[i]);
            i++;
            j++;
        }
        else if(vTimestampsLeft[i] > vTimestampsRight[j]){
            cerr << "left timestamp = " << setiosflags(ios::fixed) << setprecision(6) << vTimestampsLeft[i] << " > ";
            cerr << setiosflags(ios::fixed) << setprecision(6) << vTimestampsRight[j] << " = right timestamp."<< endl;
            j++;
            nwrongmatch++;
        }
        else{
            cerr << "left timestamp = " << setiosflags(ios::fixed) << setprecision(6) << vTimestampsLeft[i] << " < ";
            cerr << setiosflags(ios::fixed) << setprecision(6) << vTimestampsRight[j] << " = right timestamp."<< endl;
            i++;
            nwrongmatch++;
        }
    }
    if(nwrongmatch != 0){
        cerr << "There are "<< nwrongmatch << " wrong matches."<< endl;
    }
    else{
        cout << "Dataset images match perfectly." << endl;
    }

    string strPrefixLeft = strPathToSequence + "/left/";
    string strPrefixRight = strPathToSequence + "/right/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setiosflags(ios::fixed) << setprecision(6) << vTimestamps[i];
        // ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

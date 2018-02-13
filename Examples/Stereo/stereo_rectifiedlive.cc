#include<iostream>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<algorithm>
#include<opencv2/core/core.hpp>
#include "System.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
using namespace std;

#ifdef COMPILEDWITHC11
#define SET_CLOCK(t0) \
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

#else

#define SET_CLOCK(t0) \
  std::chrono::monotonic_clock::time_point t0 = std::chrono::monotonic_clock::now();
#endif
#define TIME_DIFF(t0, t1) \
(std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())

int main(int argc, char **argv) {
    if (argc != 4)
    {  
      cerr<<argv[0]<<"required path to vocabulary;path to settings;number of seconds";
    }

    cv::VideoCapture vcap;
    cv::Mat image;

    const std::string videoStreamAddress = "http://192.168.0.100:9090/stream/video.mjpeg";
    /* it may be an address of an mjpeg stream, 
    e.g. "http://user:pass@cam_address:8081/cgi/mjpg/mjpg.cgi?.mjpg" */

    //check if the videostream is open
    if(!vcap.open(videoStreamAddress)) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true); // argv[1] is the vocabulary file and argv[2] is the settings 
    double timetorun=atof(argv[3]);
    int nof=0;
    SET_CLOCK(t0);
    
     // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    cout <<"Start processing sequence..."<<endl;
    cout<<"Reading input images from the video stream..."<<endl;
    cout<<"Reading input frame by frame..."<<endl;
    
    //Create output window for displaying frames. 
    //It's important to create this window outside of the `for` loop
    //Otherwise this window will be created automatically each time you call
    //`imshow(...)`, which is very inefficient. 
    
    cv::namedWindow("Output Window");
    cv::namedWindow("Left");
    cv::namedWindow("Right");
    cout<<"cap is opened"<<endl;
    for(;;) {
        if(!vcap.read(image)) {
            std::cout << "No frame" << std::endl;
            cv::waitKey();
        }
        vcap.read(image);
        cv::Mat imgL = image(cv::Range(0, image.rows), cv::Range(0, image.cols / 2));
        cv::Mat imgR = image(cv::Range(0, image.rows), cv::Range(image.cols / 2 + 1, image.cols));
        cv::imshow("Output Window", image);
        cv::imshow("Left",imgL);
        cv::imshow("Right",imgR);
        cv::Mat imLeft;
        cv::Mat imRight;
        cv::remap(imgL,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imgR,imRight,M1r,M2r,cv::INTER_LINEAR);
        //setting the second clock
        SET_CLOCK(t1);
        double tframe=TIME_DIFF(t0,t1);
        if(tframe>timetorun)
        {
          break;
        }
        SLAM.TrackStereo(imLeft,imRight,tframe);
        nof++;
        if(cv::waitKey(1) >= 0) break;
    }
    SLAM.Shutdown();
    cout<<"Mean Tracking time:"<<nof/timetorun<<endl;
    //Saving the keyframe trajectories
    SLAM.SaveKeyFrameTrajectoryTUM("Trajectory_stereo.txt");   
    return 0;
}

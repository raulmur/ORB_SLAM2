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

        //setting the second clock
        SET_CLOCK(t1);
        double tframe=TIME_DIFF(t0,t1);
        if(tframe>timetorun)
        {
          break;
        }
        SLAM.TrackStereo(imgL,imgR,tframe);
        nof++;
        if(cv::waitKey(1) >= 0) break;
    }
    SLAM.Shutdown();
    cout<<"Mean Tracking time:"<<nof/timetorun<<endl;
    //Saving the keyframe trajectories
    SLAM.SaveKeyFrameTrajectoryTUM("Trajectory_stereo.txt");   
    return 0;
}

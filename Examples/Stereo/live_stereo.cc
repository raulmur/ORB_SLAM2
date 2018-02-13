#include<iostream>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<algorithm>
#include<opencv2/core/core.hpp>
#include "System.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

#ifdef COMPILEDWITHC11
#define SET_CLOCK(t0) \
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

#else

#define SET_CLOCK(t0) \
  std::chrono::monotonic_clock::time_point t0 = std::chrono::monotonic_clock::now();
#endif
#define TIME_DIFF(t0, t1) \
  (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())

int main(int argc, char ** argv)
{
  //check for the command-line arguments
  if(argc!=4)
  {
    cerr<<argv[0]<<"path to vocabulary; path to settings file; time to run";
  }
  
  //to capture video
  VideoCapture cap;
  Mat image;
  const std::string videoStreamAddress="http://192.168.0.100:9090/stream/video.mpeg";
  if(!cap.open(videoStreamAddress))
  {
    cout<<"Error opening the video stream or file"<<endl;
  }
  
  //initialization for slam
  ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);
  double timetorun=atof(argv[3]);
  
  //initializing number of frames to 0
  int nof=0;
  
  //setting initial clock time
  SET_CLOCK(t0);
  cout <<"Start processing sequence..."<<endl;
  cout<<"Reading input images from the video stream..."<<endl;
  cout<<"Reading input frame by frame..."<<endl;

  for(;;)
  {
    if(!cap.read(image)){
      cout<<"No frames"<<endl;
      waitKey();
    }
    //read the image
    cap.read(image);
    //divide the image into 2
    Mat imgL = image(cv::Range(0, image.rows), cv::Range(0, image.cols / 2));
    Mat imgR = image(cv::Range(0, image.rows), cv::Range(image.cols / 2 + 1, image.cols));
    SET_CLOCK(t1);
    double tframe=TIME_DIFF(t0,t1);
    if(tframe>timetorun)
    {
       break;
    }
   
    //input to SLAM
    SLAM.TrackStereo(imgL,imgR,tframe);
    nof++;
  }
  
  SLAM.Shutdown();
  cout<<"Mean Tracking time:"<<nof/timetorun<<endl;
  //Saving the keyframe trajectories
  SLAM.SaveKeyFrameTrajectoryTUM("Trajectory_stereo.txt");   
  return 0;
}

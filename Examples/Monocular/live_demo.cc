#include<iostream>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<algorithm>
#include<opencv2/core/core.hpp>
#include "System.h"

using namespace std;


#ifdef COMPILEDWITHC11
#define SET_CLOCK(t0)
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

#else

#define SET_CLOCK(t0)
  std::chrono::monotonic_clock::time_point t0 = std::chrono::monotonic_clock::now();
#endif
#define TIME_DIFF(t0, t1)
  std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0).count();

int main(int argc, char **argv[])
{
  //Check if the command-line arguments are given correctly
  if (!argc=4)
  {  
    cerr<<argv[0]<<"required path to vocabulary;path to settings;number of seconds";
  }

  //Capture video from webcam
  VideoCapture cap(0);
  if(!cap.isOpened())
  {
    //Throw an error message if there is no video stream
    cout<<"Video not available.. Video stream not available";
  }
  //initializing threads for the slam process
  ORB_SLAM2::System SLAM(argv[1],argv[2],ORBSLAM2::System::MONOCULAR,true); // argv[1] is the vocabulary file and argv[2] is the settings file 
  cv::Mat frame;
  timetorun=atof(argv[3]);
  //Initialising number of frames to 0
  int nof=0;
  SET_CLOCK(t0); 
  while(true)
  {
     cap.grab();
     cap>>frame;
     SET_CLOCK(t1);
     double tframe=TIME_DIFF(t0,t1);
     if(tframe>timetorun)
     {
       break;
     }
     cout <<"Start processing sequence..."<<endl;
     cout<<"Reading input images from the video stream..."<<endl;
     cout<<"Reading input frame by frame..."<<endl;
     //Start the slam process
     SLAM.TrackMonocular(frame,tframe);
     SET_CLOCK(t2);
     double tdf=TIME_DIFF(t1,t2);
     //To get a count of the number of frames
     nof++; 
  }

  SLAM.Shutdown();
  cout<<"Mean Tracking time:"<<nof/timetorun<<endl;
  //Saving the keyframe trajectories
  SLAM.SaveKeyFrameTrajectoryTUM("Trajectory.txt");   
  return 0;
}


//if(frame.empty())
//{
//cout<<"No frames available";
//}
//else
//{



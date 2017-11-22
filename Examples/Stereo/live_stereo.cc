nclude<iostream>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<algorithm>
#include<opencv2/core/core.hpp>
#include "System.h"

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

int main(int argc, char **argv)
{
  //Check if the command-line arguments are given correctly
  if (argc != 4)
  {
    cerr<<argv[0]<<"required path to vocabulary;path to settings;number of seconds";
  }

  //Capture video from webcam
  cv::VideoCapture cap;
  const std::string videoStreamAddress = "http://10.201.131.67:9090/stream/video.mjpeg";
  if(!vcap.open(videoStreamAddress))
  {
    //Throw an error message if there is no video stream
    cout<<"Video not available.. Video stream not available";
    return -1;
  }
  //initializing threads for the slam process
  ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true); // argv[1] is the vocabulary file and argv[2] is the settings file 
  cv::Mat image;
  double timetorun=atof(argv[3]);
  //Initialising number of frames to 0
  int nof=0;
  SET_CLOCK(t0);
  cout <<"Start processing sequence..."<<endl;
  cout<<"Reading input images from the video stream..."<<endl;
  cout<<"Reading input frame by frame..."<<endl;
  cout<<"cap is opened"<<endl;
  cv2.namedWindow("Output Window");
  while(true)
  {
     for(;;)
     {
        if(!cap.read(image)) {
            std::cout << "No frame" << std::endl;
            cv::waitKey();
     }
     //Mat croppedFrame = frame(Rect(0, frame.rows/2, frame.cols, frame.rows/2));
     cv::Mat imgL = image(cv::Range(0, image.rows), cv::Range(0, image.cols / 2));
     cv::Mat imgR = image(cv::Range(0, image.rows), cv::Range(image.cols / 2 + 1, image.cols));
     cv::imshow("Output Window", image);
     SET_CLOCK(t1);
     double tframe=TIME_DIFF(t0,t1);
     if(tframe>timetorun)
     {
       break;
     }

     //Start the slam process
     SLAM.TrackStereo(imgL,imgR,tframe);
     SET_CLOCK(t2);
     //double tdf=TIME_DIFF(t1,t2);
     //To get a count of the number of frames
     nof++;
  }
  if(cv::waitKey(1) >= 0) break;
  SLAM.Shutdown();
  cout<<"Mean Tracking time:"<<nof/timetorun<<endl;
  //Saving the keyframe trajectories
  SLAM.SaveKeyFrameTrajectoryTUM("Trajectory.txt");
  return 0;
}


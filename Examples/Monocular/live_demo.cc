#include<iostream>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<algorithm>
#include<opencv2/core/core.hpp>
#include "System.h"

using namespace std;

int main(int argc, char **argv)
{

VideoCapture cap(0);
if(!cap.isOpened())
{
  cout<<"Video not available";
}
else
{
  Mat frame;
  while(true){
     cap.grab();
     cap>>frame;

//Creating a SLAM system 
//ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

//cout <<"Start processing sequence..."<<endl;
//cout<<"Reading input images from the video stream..."<<endl;
//cout<<"Reading input frame by frame..."<<endl;

//if(frame.empty())
//{
//cout<<"No frames available";
//}
//else
//{



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

#ifdef _MSC_VER
#include <boost/config/compiler/visualc.hpp>
#endif
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>
#include<opencv2/core/core.hpp>
#include <vector>
#include <map>
#include<System.h>

using namespace std;
using boost::property_tree::ptree;
void show_interesting_object(std::map<long unsigned int, std::vector<ORB_SLAM2::Traficsign> > &image_trafficsigns_map);
bool ExtractSemanticObjGrp(std::string jsonFilename,std::map<long unsigned int, std::vector<ORB_SLAM2::Traficsign> > &SemanticObjGrp);
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int gImgWidth=1280;
int gImgHeight=720;
int gMinRectWidth = 90;
int gMinRectHeight = 90;


int main(int argc, char** argv)
{
	
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_vocabulary path_to_settings path_to_sequence path_to_jsonfile" << endl;
        return 1;
    }
	std::map<long unsigned int, std::vector<ORB_SLAM2::Traficsign> > Trafic;


    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

	
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
	if((argc >= 5) && (true == ExtractSemanticObjGrp(argv[4],Trafic)))
	{
		SLAM.SetSemanticObjGrp(Trafic);
	}
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }
		double ttrack = (double)cv::getTickCount();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);
		
		ttrack = 1000 * (((double)cv::getTickCount() - ttrack) / cv::getTickFrequency());
		vTimesTrack[ni]=ttrack;

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
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;

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
void TransformRect(std::vector<double> RectArr,cv::Rect& Roi)
{
	int ymin = int(RectArr[0] * gImgHeight);
    int xmin = int(RectArr[1] * gImgWidth);
    int ymax = int(RectArr[2] * gImgHeight);
    int xmax = int(RectArr[3] * gImgWidth);
	Roi.x = xmin;
	Roi.y = ymin;
	Roi.width = xmax - xmin;
	Roi.height = ymax - ymin;
}

void enlarge_rectangle(cv::Rect& rectangle)
{
	if (rectangle.width >= gMinRectWidth && rectangle.height >= gMinRectHeight)
	{
		return;
	}
	if (rectangle.width < gMinRectWidth)
	{	
		int total_x_displacement = gMinRectWidth - rectangle.width;
		int x_displacement = floor(total_x_displacement / 2);

		if ( ((rectangle.x - x_displacement) >= 0) && ((rectangle.x+rectangle.width+x_displacement) < gImgWidth) )
		{
			rectangle.x -= x_displacement;
			rectangle.width = gMinRectWidth;
		}
		else if((rectangle.x+gMinRectWidth) <= gImgWidth)
			rectangle.width = gMinRectWidth;
	}
	if (rectangle.height < gMinRectHeight) 
	{
		int total_y_displacement = gMinRectHeight - rectangle.height;
		int y_displacement = floor(total_y_displacement / 2);
		if ( ((rectangle.y - y_displacement) >= 0) && ((rectangle.y+rectangle.height+y_displacement) < gImgHeight) )
		{
			rectangle.y -= y_displacement;
			rectangle.height = gMinRectHeight;
		}
		else if((rectangle.y+gMinRectHeight) <= gImgHeight)
			rectangle.height = gMinRectHeight;
	}
}
bool ExtractSemanticObjGrp(std::string jsonFilename,std::map<long unsigned int, std::vector<ORB_SLAM2::Traficsign> > &SemanticObjGrp)
{
	boost::property_tree::ptree pt;
   	std::fstream jsonfile(jsonFilename);
	if(false == jsonfile.is_open())
	{
		cout<<"Unable to open json file"<<endl;
		return false;
	}
	
	boost::property_tree::read_json(jsonfile, pt);
	jsonfile.close();
	
	
   for (ptree::iterator pt_iter = pt.begin(); pt_iter != pt.end(); pt_iter++)
   {
      std::string image_name = pt_iter->first; 
      auto &traffic_sign_arr = pt_iter->second;
      std::vector<ORB_SLAM2::Traficsign> traffic_signs;
      BOOST_FOREACH(boost::property_tree::ptree::value_type &node, traffic_sign_arr.get_child("traffic_signs"))
      {
         ORB_SLAM2::Traficsign t;
         t.classid = node.second.get<int>("class_id");
         t.confidence = node.second.get<double>("confidence");
         std::vector<double> r;
         for (auto &temppt : node.second.get_child("rectangle"))
         {	
			r.push_back(temppt.second.get_value < double > ());	 
         }	
		TransformRect(r,t.Roi);		 
         //t.Roi.x = r[0];
         //t.Roi.y = r[1];
         //t.Roi.width = r[2]-r[0];
         //t.Roi.height = r[3]-r[1];
		
		 enlarge_rectangle(t.Roi);
		 //if( (t.Roi.x == 842) && (t.Roi.y == 166) )
         traffic_signs.push_back(t);

      }
	
      SemanticObjGrp.insert({stoul(image_name), traffic_signs });
   }
   
   return true;;
}

void show_interesting_object(std::map<long unsigned int, std::vector<ORB_SLAM2::Traficsign> > &image_trafficsigns_map)
{
	
   for (auto &map_item : image_trafficsigns_map)
   {

      std::cout << "image- " << map_item.first << ":" << std::endl;
      for (auto &vector_item : map_item.second)
      {
         std::cout << "\ttraffic_signs: " << std::endl;
         std::cout << "\t\tclass_id- "<< vector_item.classid << std::endl;
         std::cout << "\t\tconfidance- "<< std::setprecision(16) << vector_item.confidence << std::endl;
         std::cout << "\t\trectangle- [";

         std::cout << vector_item.Roi.x << "  ";
         std::cout << vector_item.Roi.y << "  ";
         std::cout << vector_item.Roi.width << "  ";
         std::cout << vector_item.Roi.height;
         std::cout << "]" << std::endl;
         std::cout << "----------------------------------------" << std::endl;
      }
   }
}
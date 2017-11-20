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

bool ExtractSemanticObjGrp(std::string jsonFilename,std::map<long unsigned int, std::vector<ORB_SLAM2::Traficsign> > &SemanticObjGrp);
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
	
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_vocabulary path_to_settings path_to_sequence path_to_jsonfile" << endl;
        return 1;
    }
	ORB_SLAM2::KeySemanticObjGrp SemanticObjGrp;
	std::map<long unsigned int, std::vector<ORB_SLAM2::Traficsign> > Trafic;
	if(true == ExtractSemanticObjGrp(argv[4],Trafic))
		SemanticObjGrp.SetSemanticObjGrp(Trafic);

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

	
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
	SLAM.SetSemanticObjGrp(SemanticObjGrp);
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
         std::vector<int> r;
         for (auto &temppt : node.second.get_child("rectangle"))
         {			 
            r.push_back(round(temppt.second.get_value < double > ()));
         }
         t.Roi.x = r[0];
         t.Roi.y = r[1];
         t.Roi.width = r[2];
         t.Roi.height = r[3];
         traffic_signs.push_back(t);
      }
      SemanticObjGrp.insert({stoul(image_name), traffic_signs });
   }
   
   return true;;
}
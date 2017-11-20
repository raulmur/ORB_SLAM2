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


#ifndef KEYSEMANTICOBJGRP_H
#define KEYSEMANTICOBJGRP_H

#include<string>
#include<opencv2/core/core.hpp>
#include <vector>
#include <map>
#include <iostream>


namespace ORB_SLAM2
{

struct Traficsign
{
   int classid;
  float confidence;
   cv::Rect Roi;
};

class KeySemanticObjGrp
{
	
	std::map<long unsigned int, std::vector<Traficsign> > mSemanticObjGrp;
	
   public:
	KeySemanticObjGrp();	
	bool GetSemanticObjects(std::vector<cv::Rect> &RoiList, long unsigned int frameid);
	void SetSemanticObjGrp(std::map<long unsigned int, std::vector<Traficsign> > &InterestedObject);
	std::map<long unsigned int, std::vector<Traficsign> >& GetSemanticObjGrp();
	bool isLoaded;
};


}// namespace ORB_SLAM

#endif // SYSTEM_H

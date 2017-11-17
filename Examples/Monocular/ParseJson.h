#pragma once
#ifdef _MSC_VER
#include <boost\config\compiler\visualc.hpp>
#endif
#include <boost\property_tree\ptree.hpp>
#include <boost\property_tree\json_parser.hpp>
#include <boost\foreach.hpp>
#include <exception>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <iomanip>
#include "opencv2/core.hpp"
#include<opencv2/core/core.hpp>
#include<System.h>
/*
struct TrafficSign
{
   int class_id;
   float confidence;
   cv::Rect Roi;
};*/

class JSONParser
{
public: 
   void ExtractInrestedObject();
   std::map<long unsigned int, std::vector<Traficsign> >& getInrestedObject();
   void show_interesting_object();
   JSONParser(std::string InFileName);

private:
   std::map<long unsigned int, std::vector<Traficsign> > image_trafficsigns_map;
   boost::property_tree::ptree pt;
   std::string jsonFilename;
};


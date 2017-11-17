#include "ParseJSON.h"
using boost::property_tree::ptree;

void JSONParser::ExtractInrestedObject()
{
   
   for (ptree::iterator pt_iter = pt.begin(); pt_iter != pt.end(); pt_iter++)
   {
      std::string image_name = pt_iter->first; 
      auto &traffic_sign_arr = pt_iter->second;
      std::vector<Traficsign> traffic_signs;
      BOOST_FOREACH(boost::property_tree::ptree::value_type &node, traffic_sign_arr.get_child("traffic_signs"))
      {
         Traficsign t;
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
      image_trafficsigns_map.insert({stoul(image_name), traffic_signs });
   }
}
std::map<long unsigned int, std::vector<Traficsign> >& JSONParser::getInrestedObject()
{
	return image_trafficsigns_map;
}
void JSONParser::show_interesting_object()
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

JSONParser::JSONParser(std::string InFileName):jsonFilename(InFileName)
{
	std::fstream jsonfile(jsonFilename);
	boost::property_tree::read_json(jsonfile, pt);
	jsonfile.close();
	ExtractInrestedObject();
}




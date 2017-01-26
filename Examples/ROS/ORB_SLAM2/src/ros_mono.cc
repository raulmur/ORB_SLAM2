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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_broadcaster.h>  
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Point32.h>
#include "std_srvs/Empty.h"

#include<opencv2/core/core.hpp>
#include <thread>


#include "System.h"
#include "Observer.h"
#include "Map.h"
#include "MapPoint.h"
#include "Converter.h"
using namespace std;

class ImageGrabber : public ORB_SLAM2::Observer
{
public:
    ros::Publisher pose_publisher ;
    ros::Publisher map_publisher ;
    ros::Publisher keyframe_publisher ;
    std::thread * map_publisher_thread = NULL ;
    std::string topic_id ;
    ORB_SLAM2::System* mpSLAM;


    ImageGrabber(std::string topic):topic_id(topic), mpSLAM(){
    }

    ImageGrabber(ORB_SLAM2::System* pSLAM, std::string topic):topic_id(topic), mpSLAM(pSLAM){
	pSLAM->addMapObserver(this);
    }

    void setSlam(ORB_SLAM2::System* pSLAM){
    	mpSLAM = pSLAM ;
    	pSLAM->addMapObserver(this);
    }


    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    bool ResetSlam(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res ){
    	mpSLAM->Reset();
	return true ;
    }
    void MapUpdated(ORB_SLAM2::Map * map);
    void DrawKeypoints(cv::Mat & im) ;

    virtual void update(){
	ORB_SLAM2::Map * sub = (ORB_SLAM2::Map *) this->getSubject();	
	if(map_publisher_thread != NULL){
		if(map_publisher_thread->joinable()) return ;
	}
	map_publisher_thread = new std::thread(&ImageGrabber::MapUpdated, this,  sub);
	(*map_publisher_thread).detach();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    ros::NodeHandle nodeHandler("~");
    std::string voc_path;
    std::string config_path;

    std::string map_topic_name ;
    std::string origin_frame_name = "initial_pose" ;
    std::string image_topic_name ;
    std::string publish_topic_id ;
    std::string publish_topic_path ;

    if(nodeHandler.getParam("voc_path", voc_path)){
   	ROS_INFO("Got 'voc_path' : %s",  voc_path.c_str());
    }else{
	ROS_ERROR("Failed to get param 'voc_path'");
	ros::shutdown();
        return 1; 
    }
    if(!nodeHandler.getParam("config_path", config_path)){
	ROS_ERROR("Failed to get param 'config_path'");
        ros::shutdown();
        return 1;
    }
    nodeHandler.param<std::string>("image_topic", image_topic_name, "/camera/image_raw");
    nodeHandler.param<std::string>("publish_topic", publish_topic_id, "camera");
    publish_topic_path = "/"+publish_topic_id+"/";

    ROS_INFO("Launching orb_slam2 mono with %s %s ", voc_path.c_str(), config_path.c_str());
    ROS_INFO("Image grabbed from : %s, publishing on id : %s ", image_topic_name.c_str(), publish_topic_id.c_str());
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(voc_path.c_str(),config_path.c_str(),ORB_SLAM2::System::MONOCULAR,false);
    ImageGrabber igb(publish_topic_id);
    igb.pose_publisher = nodeHandler.advertise<geometry_msgs::PoseStamped>(publish_topic_path+"pose", 100);
    igb.map_publisher = nodeHandler.advertise<sensor_msgs::PointCloud>(publish_topic_path+"map", 100);
    igb.keyframe_publisher = nodeHandler.advertise<sensor_msgs::Image>(publish_topic_path+"keyframe", 100);
    igb.setSlam(&SLAM);

    ros::Subscriber sub = nodeHandler.subscribe(image_topic_name, 1, &ImageGrabber::GrabImage,&igb);
    ros::ServiceServer service = nodeHandler.advertiseService("reset", &ImageGrabber::ResetSlam, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}



void ImageGrabber::DrawKeypoints(cv::Mat & im){
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    ORB_SLAM2::Tracking * pTracker = mpSLAM->getTracker();
    vCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    unsigned int N = vCurrentKeys.size();
vbVO.resize(N);
vbMap.resize(N);
    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);
	



    if(pTracker->mState ==ORB_SLAM2::Tracking::OK) //TRACKING
    {
	for(unsigned int i=0;i<N;i++)
	{
		ORB_SLAM2::MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
		vbMap[i] = false ;
		vbVO[i]=false;
		if(pMP)
		{
			if(!pTracker->mCurrentFrame.mvbOutlier[i])
			{
			    if(pMP->Observations()>0)
				vbMap[i]=true;
			    else
				vbVO[i]=true;
			}
		}
	}
        const float r = 5;
        for(unsigned int i=0;i<N;i++)
        {
                if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                }
            }
        }
    }

}


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat mTcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec()); // cam to world expressed in world frame ?
    cv::Mat overlayed = cv_ptr->image.clone();
    DrawKeypoints(overlayed);
    if(!mTcw.empty()){
	cv::Mat Rcw = mTcw.rowRange(0,3).colRange(0,3); //world to cam expressed in world frame ?
   	cv::Mat tcw = mTcw.rowRange(0,3).col(3); //world to cam expressed in world frame ?
    	tf::Matrix3x3 M(Rcw.at<float>(0,0),Rcw.at<float>(0,1),Rcw.at<float>(0,2),
                        Rcw.at<float>(1,0),Rcw.at<float>(1,1),Rcw.at<float>(1,2),
                        Rcw.at<float>(2,0),Rcw.at<float>(2,1),Rcw.at<float>(2,2));
    	tf::Vector3 V(tcw.at<float>(0), tcw.at<float>(1), tcw.at<float>(2));
    	tf::Transform tfTcw(M,V);
    	static tf::TransformBroadcaster mTfBr;
    	mTfBr.sendTransform(tf::StampedTransform(tfTcw, ros::Time::now(), topic_id, "initial_pose"));
	vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rcw);
	geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "initial_pose"; //need to have reference frame to be passed as an argument
        pose_stamped.header.stamp = ros::Time::now();

        pose_stamped.pose.orientation.x = q[0];
        pose_stamped.pose.orientation.y = q[1];
        pose_stamped.pose.orientation.z = q[2];
        pose_stamped.pose.orientation.w = q[3];

        pose_stamped.pose.position.x = mTcw.at<float>(0, 3);
        pose_stamped.pose.position.y = mTcw.at<float>(1, 3);
        pose_stamped.pose.position.z = mTcw.at<float>(2, 3);
	this->pose_publisher.publish(pose_stamped);
    }

	cv_bridge::CvImage out_msg;
	out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
	out_msg.image    = overlayed; // Your cv::Mat

	keyframe_publisher.publish(out_msg.toImageMsg());

}

//Should be called outside the scope of ORB_SLAM2
void ImageGrabber::MapUpdated(ORB_SLAM2::Map * map){
	//TODO: implement new keypoints retrieval and publish a point cloud
	sensor_msgs::PointCloud pcl_map;
	pcl_map.header.frame_id = "initial_pose"; //need to have reference frame to be passed as an argument
	pcl_map.header.stamp = ros::Time::now();
	std::vector<ORB_SLAM2::MapPoint*> map_points = map->GetReferenceMapPoints();
	//std::vector<ORB_SLAM2::MapPoint*> map_points = map->GetAllMapPoints();
	for (unsigned int i = 0 ; i < map_points.size() ; i ++){
		geometry_msgs::Point32 mapPoint ;
		cv::Mat pose = map_points[i]->GetWorldPos();
		mapPoint.x = pose.at<float>(0, 0);
		mapPoint.y = pose.at<float>(1, 0);
		mapPoint.z = pose.at<float>(2, 0);
		pcl_map.points.push_back(mapPoint) ;
	}
	//ROS_INFO("Received map update !!!");
	map_publisher.publish(pcl_map);

}


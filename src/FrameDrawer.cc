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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    vector<bool> vbUselessPoint; // Useless MapPoints in current frame(outliers)
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
            vbUselessPoint = mvbUselessPoint;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
            vbUselessPoint = mvbUselessPoint;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    // Now draw the good and bad descriptors
    int nGoodDescriptor = mvGoodDescriptor.size();
    for(int i=0; i<nGoodDescriptor; i++){
        cv::Point2f pt1,pt2;
        pt1.x=mvGoodDescriptor[i].pt.x-mvGoodDescriptorRadius[i];
        pt1.y=mvGoodDescriptor[i].pt.y-mvGoodDescriptorRadius[i];
        pt2.x=mvGoodDescriptor[i].pt.x+mvGoodDescriptorRadius[i];
        pt2.y=mvGoodDescriptor[i].pt.y+mvGoodDescriptorRadius[i];
        cv::rectangle(im,pt1,pt2,cv::Scalar(255,255,0));
        cv::circle(im,mvGoodDescriptor[i].pt,1,cv::Scalar(255,255,0),-1);
    }
    int nBadDescriptor = mvBadDescriptor.size();
    for(int i=0; i<nBadDescriptor; i++){
        cv::Point2f pt1,pt2;
        pt1.x=mvBadDescriptor[i].pt.x-mvBadDescriptorRadius[i];
        pt1.y=mvBadDescriptor[i].pt.y-mvBadDescriptorRadius[i];
        pt2.x=mvBadDescriptor[i].pt.x+mvBadDescriptorRadius[i];
        pt2.y=mvBadDescriptor[i].pt.y+mvBadDescriptorRadius[i];
        cv::rectangle(im,pt1,pt2,cv::Scalar(0,0,128));
        cv::circle(im,mvBadDescriptor[i].pt,1,cv::Scalar(0,0,128),-1);
    }

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        mnUselessPoint=0; // For debug use
        mnDiscardedPoint=0; // For debug use
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
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
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0)); //BGR
                    cv::circle(im,vCurrentKeys[i].pt,1,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,1,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
            else{
                cv::circle(im,vCurrentKeys[i].pt,0,cv::Scalar(0,0,255),-1);
                mnUselessPoint++;
            }
            if(mvbDiscardedPoint[i]){ // For debug use
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
                cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,255));
                cv::circle(im,vCurrentKeys[i].pt,1,cv::Scalar(255,0,255),-1);
                mnDiscardedPoint++;
            }
        }
        
    }
    else if(mState==Tracking::LOST){
        mnTracked=0;
        mnTrackedVO=0;
        mnUselessPoint=0; // For debug use
        mnDiscardedPoint=0; // For debug use
        const float r = 5;
        const int n = vCurrentKeys.size();

        // Now draw the good and bad descriptors
        int nGoodDescriptor = mvGoodDescriptor.size();
        for(int i=0; i<nGoodDescriptor; i++){
            cv::Point2f pt1,pt2;
            pt1.x=mvGoodDescriptor[i].pt.x-mvGoodDescriptorRadius[i];
            pt1.y=mvGoodDescriptor[i].pt.y-mvGoodDescriptorRadius[i];
            pt2.x=mvGoodDescriptor[i].pt.x+mvGoodDescriptorRadius[i];
            pt2.y=mvGoodDescriptor[i].pt.y+mvGoodDescriptorRadius[i];
            cv::rectangle(im,pt1,pt2,cv::Scalar(255,255,0));
            cv::circle(im,mvGoodDescriptor[i].pt,1,cv::Scalar(255,255,0),-1);
        }
        int nBadDescriptor = mvBadDescriptor.size();
        for(int i=0; i<nBadDescriptor; i++){
            cv::Point2f pt1,pt2;
            pt1.x=mvBadDescriptor[i].pt.x-mvBadDescriptorRadius[i];
            pt1.y=mvBadDescriptor[i].pt.y-mvBadDescriptorRadius[i];
            pt2.x=mvBadDescriptor[i].pt.x+mvBadDescriptorRadius[i];
            pt2.y=mvBadDescriptor[i].pt.y+mvBadDescriptorRadius[i];
            cv::rectangle(im,pt1,pt2,cv::Scalar(0,0,128));
            cv::circle(im,mvBadDescriptor[i].pt,1,cv::Scalar(0,0,128),-1);
        }

        for(int i=0;i<n;i++)
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
                    cv::circle(im,vCurrentKeys[i].pt,1,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,1,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
            else{
                cv::circle(im,vCurrentKeys[i].pt,0,cv::Scalar(0,0,255),-1);
                mnUselessPoint++;
            }
            if(mvbDiscardedPoint[i]){ // For debug use
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
                cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,255));
                cv::circle(im,vCurrentKeys[i].pt,1,cv::Scalar(255,0,255),-1);
                mnDiscardedPoint++;
            }
        }
        
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
        if(mnUselessPoint>0)
            s << ", # of outliers: " << mnUselessPoint;
        if(mnDiscardedPoint>=0)
            s << ", # of discared matches: " << mnDiscardedPoint;
    }
    else if(nState==Tracking::LOST)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
        if(mnUselessPoint>0)
            s << ", # of outliers: " << mnUselessPoint;
        if(mnDiscardedPoint>=0)
            s << ", # of discared matches: " << mnDiscardedPoint;

        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mvbUselessPoint = vector<bool>(N,false);
    mvbDiscardedPoint = vector<bool>(N,false); // For debug use
    mbOnlyTracking = pTracker->mbOnlyTracking;
    mvBadDescriptor = pTracker->mCurrentFrame.mvBadDescriptor;  // For debug use, I'm trying to draw those points on the FrameDrawer that does not fit the DescriptorDistance requirement.
    mvBadDescriptorRadius = pTracker->mCurrentFrame.mvBadDescriptorRadius;  // For debug use, I'm trying to draw those points on the FrameDrawer that does not fit the DescriptorDistance requirement.
    mvGoodDescriptor = pTracker->mCurrentFrame.mvGoodDescriptor;  // For debug use.
    mvGoodDescriptorRadius = pTracker->mCurrentFrame.mvGoodDescriptorRadius;  // For debug use.


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
        for(int i=0;i<N;i++) // For debug use
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                mvbUselessPoint[i]=true;
            }
        }
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
                else{
                    mvbUselessPoint[i]=true;  // For debug use
                }
                if(pTracker->mCurrentFrame.mvbDiscarded[i]){ // For debug use
                    mvbDiscardedPoint[i]=true;
                }
            }
        }
    }
    else{
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(pTracker->mCurrentFrame.mvbOutlier[i])  // For debug use
                {
                    mvbUselessPoint[i]=true;
                }

                if(pTracker->mCurrentFrame.mvbDiscarded[i]){ // For debug use
                    mvbDiscardedPoint[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM

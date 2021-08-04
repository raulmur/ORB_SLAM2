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


#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
         
#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"
         
#include "Optimizer.h"
#include "PnPsolver.h"

#include "pointcloudmapping.h"

#include <iostream>

#include <mutex>


using namespace std;

namespace ORB_SLAM2
{
Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    // 自己添加的，获取img的width和height
    int img_width = fSettings["Camera.width"];
    int img_height = fSettings["Camera.height"];

    cout << "img_width = " << img_width << endl;
    cout << "img_height = " << img_height << endl;

    // 自己添加的，在这里先得到映射值
    initUndistortRectifyMap(mK, mDistCoef, Mat_<double>::eye(3,3), mK, Size(img_width, img_height), CV_32F, mUndistX, mUndistY);

    cout << "mUndistX size = " << mUndistX.size << endl;
    cout << "mUndistY size = " << mUndistY.size << endl;

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}


Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, Map *pCornerMap, KeyFrameDatabase* pKFDB,CornerFrameDatebase * pCFDB,KeyFrameDatabase* pCKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap),
    mnLastRelocFrameId(0),mpCornerFrameDB(pCFDB),mpCornerMap(pCornerMap), mpCornerKeyFrameDB(pCKFDB)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(mDepthMapFactor==0)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImRGB = imRGB;
    mImGray = imRGB;
    mImDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    //std::cout << "mDepthMapFactor=" << mDepthMapFactor << std::endl;
    //std::cout << "mImDepth.type()" << mImDepth.type() << endl;
    if(mDepthMapFactor!=1 || mImDepth.type()!=CV_32F)
    {
        mImDepth.convertTo(mImDepth, CV_32F, mDepthMapFactor);
    }

    mCurrentFrame = Frame(mImGray, mImDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageMonocularWithLine(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;
//    mNormal = normal;
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    //TODO 纠正畸变
    //cv::remap(mImGray, mImGray, mUndistX, mUndistY, cv::INTER_LINEAR);
    //mImGray = mImGray(Rect(15, 15, 610, 450));

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,mNormal,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,mNormal,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    //Track();
    TrackWithLine();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::TrackWithLine()
{

    // Track包含两部分：估计运动、跟踪局部地图

    // mState为tracking的状态机
    // SYSTEM_NOT_READY=-1, NO_IMAGES_YET=0, NOT_INITIALIZED=1, OK=2, LOST=3
    // 如果图像复位过，或者第一次运行，则为NO_IMAGES_YET状态
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }
    bool bIni_Manhattan = true;
    bool bManhattanGood = false;
    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if (!bIni_Manhattan)
            MonocularInitialization(); // ORBSLAM Initialization
        else
            MonocularInitializationWithLine(); //Manhattan initialization

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else  //完成初始化之后
    {
        //Tracking: system is initialized
        // bOK是临时变量，用于表示每个函数是否执行成功
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {


            cv::Mat relatedPose = cv::Mat::zeros(cv::Size(3,3),CV_32F);
            if(bIni_Manhattan)
            {
                // compute rotation based on the Manhattan
                cv ::Mat MF_can= cv::Mat::zeros(cv::Size(3,3),CV_32F);
                cv ::Mat MF_can_T= cv::Mat::zeros(cv::Size(3,3),CV_32F);
                cv ::Mat mRotation_wc1= cv::Mat::zeros(cv::Size(3,3),CV_32F);

                MF_can=TrackManhattanFrame(mLastRcm,mCurrentFrame.vSurfaceNormal,mCurrentFrame.mVF3DLines).clone();
                //cv::Mat R_cm=ClusterMMF(MF_can);
                //MF_can_T = mLastRcm.t();
                //mRotation_wc=Rotation_cm*MF_can_T;
                //mRotation_wc1=mRotation_wc.t();  //R_k-1,1


                MF_can_T=MF_can.t();
                mRotation_wc=Rotation_cm*MF_can_T; //R_1,k
                mRotation_cw=mRotation_wc.t();   //R_k,1

                relatedPose=MF_can*mLastRcm.t();
                MF_can.copyTo(mLastRcm);
                //relatedPose = (mRotation_wc1*mRotation_wc).t(); //  R_k-1,k -->R_k,k-1

            }

            if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
            {
                //bOK = TrackReferenceKeyFrame();
            }
            else
            {
                if (bIni_Manhattan&&TranslationWithMotionModel(relatedPose))
                {
                    cout<<"\t translation only"<<endl;
                    bManhattanGood = true;
                }
                else
                {

                    bOK = TrackReferenceKeyFrame();
                    if(bOK)  bManhattanGood = true;

                }

                if(!bManhattanGood)
                {
                    bOK = Relocalization();
                    if(bOK) bManhattanGood= true;
                }


            }



            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();


                //compute translation
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    cout<<"tracking: in reference"<<endl;
                    bOK = TrackReferenceKeyFrame();

                }
                else
                {

                    //else
                    {
                        if(!bManhattanGood)
                        bOK = TrackWithMotionModel();
                        //fix manhattan pose
                        //UpdateManhattanPose();
                    }

                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }

        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

//        ofstream file("TrackLocalMapTime.txt", ios::app);
//        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        //-----------------------------------step 2.2-------------------------------------------------
        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
                cout<<"Tracking localmap with Lines"<<endl;
                bOK=TrackLocalMap();
                if(!bOK)
                    bOK = TrackLocalMapWithLines();
            }
            else
            {
                bOK=Relocalization();
            }
        }
        cout<<"Tracking: finish localmap"<<endl;

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        cout<<"tracking: check if a keyframe "<<endl;
        if(bOK)
        {
            //cout<<" tracking: Clean VO matches0"<<endl;
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {

                // 更新恒速运动模型TrackWithMotionModel中的mVelocity
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);

                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));

                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));

                mVelocity = mCurrentFrame.mTcw*LastTwc;

            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
            //cout<<" tracking: Clean VO matches1"<<endl;
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }
            for(int i=0; i<mCurrentFrame.NL; i++)
            {
                MapLine* pML = mCurrentFrame.mvpMapLines[i];
                if(pML)
                    if(pML->Observations()<1)
                    {
                        mCurrentFrame.mvbLineOutlier[i] = false;
                        mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
                    }
            }
            //cout<<" tracking: Clean VO matches2"<<endl;
            // Delete temporal MapPoints
            // 只用于双目或rgbd
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            for(list<MapLine*>::iterator lit = mlpTemporalLines.begin(), lend =  mlpTemporalLines.end(); lit!=lend; lit++)
            {
                MapLine* pML = *lit;
                delete pML;
            }
            //cout<<" tracking: Clean VO matches3"<<endl;
            mlpTemporalPoints.clear();
            mlpTemporalLines.clear();
            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
            {
                cout << endl << "Tracking create new kf " << endl;
                CreateNewKeyFrame();
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            // 剔除那些在BA中检测为outlier的3D map点
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }

            for(int i=0; i<mCurrentFrame.NL; i++)
            {
                if(mCurrentFrame.mvpMapLines[i] && mCurrentFrame.mvbLineOutlier[i])
                    mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
            }
        }
        cout<<" tracking: Clean1"<<endl;
        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
        //cout<<"Tracking Tcr:"<<Tcr<<endl;
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}

void Tracking::Track()
{
        // Track包含两部分：估计运动、跟踪局部地图

        // mState为tracking的状态机
        // SYSTEM_NOT_READY=-1, NO_IMAGES_YET=0, NOT_INITIALIZED=1, OK=2, LOST=3
        // 如果图像复位过，或者第一次运行，则为NO_IMAGES_YET状态
        if(mState==NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED;
        }

        mLastProcessedState=mState;

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        if(mState==NOT_INITIALIZED)
        {
            if(mSensor==System::STEREO || mSensor==System::RGBD)
                StereoInitialization();
            else
                MonocularInitialization();

            mpFrameDrawer->Update(this);

            if(mState!=OK)
                return;
        }

        else  //完成初始化之后
        {
            //Tracking: system is initialized
            // bOK是临时变量，用于表示每个函数是否执行成功
            bool bOK;

            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            if(!mbOnlyTracking)
            {
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.
                // 正常初始化成功
                if(mState==OK)
                {
                    // Local Mapping might have changed some MapPoints tracked in last frame
                    // 检查并更新上一帧被替换的MapPoints, 修改添加MapLines
                    CheckReplacedInLastFrame();
                    cout << "Tracking: Initialized:mVelocity size = " << mVelocity.size << endl;

                    if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                    {

                        bOK = TrackReferenceKeyFrame();
                        cout << "Tracking: finish ReferenceKeyFrame() " << endl;
                    }
                    else
                    {


                        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                        bOK = TrackWithMotionModel();
                        cout<<"Tracking: finish motion model() bOK"<<bOK<<endl;

                        if(!bOK)
                        {
                            bOK = TrackReferenceKeyFrame();
                            cout<<"Tracking: bok:"<<bOK<<", finish ReferenceKeyFrame()"<<endl;
                        }

                        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
                        cout << "Track time: " << time_used.count() << endl;
                    }
                }
                else
                {
                    bOK = Relocalization();
                    cout<<"Tracking: Relocalization()"<<endl;
                }
            }

            mCurrentFrame.mpReferenceKF = mpReferenceKF;

//        ofstream file("TrackLocalMapTime.txt", ios::app);
//        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
            //-----------------------------------step 2.2-------------------------------------------------
            // If we have an initial estimation of the camera pose and matching. Track the local map.
            if(!mbOnlyTracking)
            {
                if(bOK)
                {
                    //cout<<"Tracking localmap with Lines"<<endl;
//                chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
                    /*if(mCurrentFrame.dealWithLine)
                        bOK = TrackLocalMapWithLines();
                    else*/
//                        bOK=TrackLocalMap();
                    bOK = TrackLocalMapWithLines();  // 原始ORB-SLAM，只有点特征
//                    bOK=TrackLocalMap();
//                chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//                chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
//                cout << "trackLocalMap time: " << time_used.count() << endl;
                }
                else
                    bOK=Relocalization();
            }
            else
            {
                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                // the camera we will use the local map again.
                if(bOK && !mbVO)
                {
                    bOK = TrackLocalMapWithLines();
//                    bOK = TrackLocalMap();      // 原始ORB-SLAM，只有点特征
                }

            }
            //-----------------------------------step 3-------------------------------------------------
            if(!mbOnlyTracking)
            {
               /* if(bOK)
                    mpCornerFrameDB->addFrame(mCurrentFrame);
                if(mpCornerFrameDB->getSizeofCornerFrames()<5)
                    cout<<"Track: adding"<<endl;
                else
                {
                    if(mpCornerFrameDB->computeCornerAngle()>50)
                    {
                        //为这几帧，构建CornerMap and related KeyFrame
                        cout<<"Tracking: begin optimization"<<endl;
                        //UpdatecornerFrames();
                        bOK=TrackLocalMapWithLines();
                        //optimize

                        //第一帧 就像 第一个关键帧
                        //buildCornerFrame()
                        //获得这些帧CornerFrame

                        //获得这些CornerMap
                        //优化当前帧
                        //trackCornerMap();
                    }
                    mpCornerFrameDB->eraseFrame();
                }*/
            }
//        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
//        file << time_used.count() << endl;
//        file.close();

            if(bOK)
                mState = OK;
            else
                mState=LOST;

            // Update drawer
            mpFrameDrawer->Update(this);

            // If tracking were good, check if we insert a keyframe
            if(bOK)
            {
                // Update motion model
                if(!mLastFrame.mTcw.empty())
                {
                    // 更新恒速运动模型TrackWithMotionModel中的mVelocity
                    cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                    mVelocity = mCurrentFrame.mTcw*LastTwc;
                }
                else
                    mVelocity = cv::Mat();

                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
                // Clean VO matches
                for(int i=0; i<mCurrentFrame.N; i++)
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                    if(pMP)
                        if(pMP->Observations()<1)
                        {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                        }
                }
                for(int i=0; i<mCurrentFrame.NL; i++)
                {
                    MapLine* pML = mCurrentFrame.mvpMapLines[i];
                    if(pML)
                        if(pML->Observations()<1)
                        {
                            mCurrentFrame.mvbLineOutlier[i] = false;
                            mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
                        }
                }
                // Delete temporal MapPoints
                // 只用于双目或rgbd
                for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
                {
                    MapPoint* pMP = *lit;
                    delete pMP;
                }
                for(list<MapLine*>::iterator lit = mlpTemporalLines.begin(), lend =  mlpTemporalLines.end(); lit!=lend; lit++)
                {
                    MapLine* pML = *lit;
                    delete pML;
                }
                mlpTemporalPoints.clear();
                mlpTemporalLines.clear();
                // Check if we need to insert a new keyframe
                if(NeedNewKeyFrame())
                {
                    cout << endl << "Tracking create new kf " << endl;
                    CreateNewKeyFrame();
                }
                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame.
                // 剔除那些在BA中检测为outlier的3D map点
                for(int i=0; i<mCurrentFrame.N;i++)
                {
                    if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                }

                for(int i=0; i<mCurrentFrame.NL; i++)
                {
                    if(mCurrentFrame.mvpMapLines[i] && mCurrentFrame.mvbLineOutlier[i])
                        mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
                }
            }

            // Reset if the camera get lost soon after initialization
            if(mState==LOST)
            {
                if(mpMap->KeyFramesInMap()<=5)
                {
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    mpSystem->Reset();
                    return;
                }
            }

            if(!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            mLastFrame = Frame(mCurrentFrame);
        }

        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if(!mCurrentFrame.mTcw.empty())
        {
            cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
            //cout<<"Tracking Tcr:"<<Tcr<<endl;
        }
        else
        {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState==LOST);
        }

}


void Tracking::MonocularInitializationWithLine()
{
    int num=50;
    cout<<"Tracking: number of points:"<<mCurrentFrame.mvKeys.size()<<endl;

    if(!mpInitializer)
    {
        if(mCurrentFrame.mvKeys.size()>num)
        {
            mInitialFrame=Frame(mCurrentFrame);
            mLastFrame=Frame(mCurrentFrame);


            //当前帧的 surface normal
            Rotation_cm=SeekManhattanFrame(mCurrentFrame.vSurfaceNormal,mCurrentFrame.mVF3DLines).clone();
            Rotation_cm=cv::Mat::zeros(cv::Size(3,3),CV_32F);
            //lr-k1
            Rotation_cm.at<float>(0,0)=0.9998;Rotation_cm.at<float>(0,1)=0.0006;Rotation_cm.at<float>(0,2)=-0.0217;
            Rotation_cm.at<float>(1,0)=-0.0010;Rotation_cm.at<float>(1,1)=0.9999;Rotation_cm.at<float>(1,2)=-0.0166;
            Rotation_cm.at<float>(2,0)=0.0217;Rotation_cm.at<float>(2,1)=0.0166;Rotation_cm.at<float>(2,2)=0.9996;
            //
            Rotation_cm.at<float>(0,0)=1.000;Rotation_cm.at<float>(0,1)=-0.0024;Rotation_cm.at<float>(0,2)=0.0009;
            Rotation_cm.at<float>(1,0)=0.0024;Rotation_cm.at<float>(1,1)=1.0000;Rotation_cm.at<float>(1,2)=0.0015;
            Rotation_cm.at<float>(2,0)=-0.0009;Rotation_cm.at<float>(2,1)=-0.0015;Rotation_cm.at<float>(2,2)=1.000;
            //of_k1
//            Rotation_cm.at<float>(0,0)=1.000;Rotation_cm.at<float>(0,1)=-0.0016;Rotation_cm.at<float>(0,2)=0.0069;
//            Rotation_cm.at<float>(1,0)=0.0016;Rotation_cm.at<float>(1,1)=1.0000;Rotation_cm.at<float>(1,2)=0.0005;
//            Rotation_cm.at<float>(2,0)=-0.0069;Rotation_cm.at<float>(2,1)=-0.0005;Rotation_cm.at<float>(2,2)=1.0000;
            //large_ca
            //Rotation_cm.at<float>(0,0)=0.7925;Rotation_cm.at<float>(0,1)=-0.0077;Rotation_cm.at<float>(0,2)=0.6099;
            //Rotation_cm.at<float>(1,0)=0.1843;Rotation_cm.at<float>(1,1)=0.9562;Rotation_cm.at<float>(1,2)=-0.2275;
            //Rotation_cm.at<float>(2,0)=-0.5814;Rotation_cm.at<float>(2,1)=0.2927;Rotation_cm.at<float>(2,2)=0.7591;

            //corridor2
//            Rotation_cm.at<float>(0,0)=0.9748;Rotation_cm.at<float>(0,1)=-0.0375;Rotation_cm.at<float>(0,2)=-0.2197;
//            Rotation_cm.at<float>(1,0)=-0.0463;Rotation_cm.at<float>(1,1)=0.9303;Rotation_cm.at<float>(1,2)=-0.3640;
//            Rotation_cm.at<float>(2,0)=0.2182;Rotation_cm.at<float>(2,1)=0.3650;Rotation_cm.at<float>(2,2)=0.9051;

            Rotation_cm.copyTo(mLastRcm);
            cv ::Mat MF_can= cv::Mat::zeros(cv::Size(3,3),CV_32F);
            cv ::Mat MF_can_T= cv::Mat::zeros(cv::Size(3,3),CV_32F);
            MF_can=TrackManhattanFrame(Rotation_cm,mCurrentFrame.vSurfaceNormal,mCurrentFrame.mVF3DLines).clone();
            MF_can.copyTo(mLastRcm);//.clone();
            //Rotation_cm.copyTo(mLastRcm);// .clone();


            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0;i<mCurrentFrame.mvKeysUn.size();i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete  mpInitializer;
            mpInitializer=new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            cout<<"Tracking: the first image"<<endl;
            return;
        }
    }
    else
    {
        cv::Mat relatedPose = cv::Mat::zeros(cv::Size(3,3),CV_32F);
        if((int)mCurrentFrame.mvKeys.size()<num)
        {
            delete mpInitializer;
            mpInitializer= static_cast<Initializer*> (NULL);
            fill(mvIniMatches.begin(), mvIniMatches.end(),-1);
            return;
        }

        ORBmatcher matcher (0.9,true);
        LSDmatcher lmatcher;   //建立线特征之间的匹配
        int nmatches =matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);
        int nlineMatches = lmatcher.SerachForInitialize(mInitialFrame, mCurrentFrame, mvLineMatches);

        cout<<"Tracking: point and line matches:"<<nmatches<<", "<<nlineMatches<<endl;
        if(nmatches<20 )
        {
            delete  mpInitializer;
            cout<<"***********restart, because of less points"<<endl;
            mpInitializer= static_cast<Initializer*>(NULL);
            return;
        }

        cv ::Mat MF_can= cv::Mat::zeros(cv::Size(3,3),CV_32F);
        cv ::Mat MF_can_T= cv::Mat::zeros(cv::Size(3,3),CV_32F);
        MF_can=TrackManhattanFrame(mLastRcm,mCurrentFrame.vSurfaceNormal,mCurrentFrame.mVF3DLines).clone();
        cout<<"TRACKING:track MF mono:"<<MF_can<<endl;
        //cv::Mat R_cm=ClusterMMF(MF_can);

        relatedPose=MF_can*mLastRcm.t(); //R21
        MF_can.copyTo(mLastRcm);
        mRotation_cw = relatedPose.t();

//        MF_can.copyTo(mLastRcm);//.clone();
//        MF_can_T=MF_can.t();
//        mRotation_wc=Rotation_cm*MF_can_T;   //mRotation_wc  就是一个 R12
//        mRotation_cw=mRotation_wc.t();




        cout<<"Tracking: finish mf rotation"<<mRotation_wc<<endl;
        //获得 Rcw
        cv::Mat Rcw= cv::Mat::zeros(cv::Size(3,3),CV_32F);; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        mRotation_cw.copyTo(Rcw);

        //Rcw 就是 R21*/




        cout<<"Tracking: initialize MW:"<<Rcw<<endl;
        if(mpInitializer->InitializeManhattanWorld(mInitialFrame,mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated, mvLineMatches, mvLineS3D, mvLineE3D, mvbLineTriangulated))
        {

            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    //cout<<vbTriangulated[i]<<endl;
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            cout<<"Tracking: nmatches"<<nmatches<<endl;

            // Set Frame Poses
            // 将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            // step6：将三角化得到的3D点包装成MapPoints
            /// 如果要修改，应该是从这个函数开始
            //CreateInitialMapMonocular();
            CreateInitialMapMonoWithLine();
        }
        cout<<"Tracking: the second image"<<endl;
    }


}

bool Tracking::TranslationWithMotionModel(cv::Mat &relatedPose)
{
    ///cout<<"Tracking: with motion model"<<endl;
    bool reliableManhattanR_wc = true;
    bool bManhattanRotation = false;
    // --step1: 建立ORB特征点的匹配
    ORBmatcher matcher(0.9,true);
    LSDmatcher lmatcher;

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    // --step2: 更新上一帧的位姿
    UpdateLastFrame();

    // --step3:根据Const Velocity Model，估计当前帧的位姿
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    // --step4：根据上一帧特征点对应的3D点投影的位置缩小特征点匹配范围
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    cout<<"translation motion model：point matches1:"<<nmatches<<endl;
    int lmatches = lmatcher.SearchByProjection(mCurrentFrame, mLastFrame);
//    int lmatches2=0;
//    if(mCurrentFrame.dealWithLine)
//    {
//        fill(mCurrentFrame.mvpMapLines.begin(),mCurrentFrame.mvpMapLines.end(), static_cast<MapLine*>(NULL));
//        vector<MapLine*> vpMapLineMatches;
//        lmatches2 = lmatcher.SearchByProjection(mpReferenceKF,mCurrentFrame,vpMapLineMatches);
//        mCurrentFrame.mvpMapLines=vpMapLineMatches;
//    }
    //vector<MapPoint*> vpMapPointMatches;

    //  int nmatches1 = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    //lmatcher.SearchByProjection(mCurrentFrame, mLastFrame.mvpMapLines, th);
    //    cout << "tracking points = " << nmatches << endl;
    //    cout << "tracking lmatches = " << lmatches << endl;

    //double lmatch_ratio = lmatches*1.0/mCurrentFrame.mvKeylinesUn.size();
    //    cout << "lmatch_ratio = " << lmatch_ratio << endl;

    // If few matches, uses a wider window search
    // 如果跟踪的点少，则扩大搜索半径再来一次
    if(nmatches<30)
    {
        cout<<"tracking: trans only: rematching"<<endl;
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,4*th,mSensor==System::MONOCULAR);
    }

//    if(mCurrentFrame.dealWithLine)
//    {
//        if(nmatches<10 &&lmatches2<3)  mCurrentFrame.SetPose(mpReferenceKF->GetPose());
//    }
//    else
//    {
//        if(nmatches<8 )
//            return false;
//    }

    // Optimize frame pose with all matches
    // --step5: 优化位姿
    cout<<"translation,pose before opti"<<mCurrentFrame.mTcw<<endl;

    //relatedPose
    if(bManhattanRotation)
        mRotation_cw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3));
    else
    {
        cout<<relatedPose<<endl;
        cout<<relatedPose.t()<<endl;
        cout<<mVelocity<<endl;
        cv::Mat poseFromLastFrame = relatedPose.t()*mLastFrame.mTcw.rowRange(0,3).colRange(0,3);
        poseFromLastFrame.copyTo(mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3));
    }

    cout<<"translation  motion model：point matches:"<<nmatches<<endl;
    //if(mCurrentFrame.dealWithLine)
    Optimizer::PoseOptimization(&mCurrentFrame, true);
    //else

    float ratio = Optimizer::TranslationOptimization(&mCurrentFrame);
    cout<<"tracking: translation motion model,pose after opti"<<mCurrentFrame.mTcw<<endl;
    cout<<"tracking: translation ratio: "<<ratio<<endl;

    //check whether this pose is good nor not
    if(ratio<0.85)
    {
        //this pose cannot reliable
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
        //fix manhattan information
        reliableManhattanR_wc = false;
        return reliableManhattanR_wc;

    }
    else
    {
        //a reliable pose
        int nmatchesMap = 0;
        int nmatchesLineMap=0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    //mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }

        for(int i=0; i<mCurrentFrame.NL;i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                if(mCurrentFrame.mvbLineOutlier[i])
                {
                    MapLine* pML=mCurrentFrame.mvpMapLines[i];
                    mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
                    mCurrentFrame.mvbLineOutlier[i]=false;
                    pML->mbTrackInView=false;
                    pML->mnLastFrameSeen=mCurrentFrame.mnId;
                    lmatches--;
                }
                else if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                    nmatchesLineMap++;
            }

        }

        mState = OK;
        return reliableManhattanR_wc;


    }

}



cv::Mat Tracking::SeekManhattanFrame(vector<SurfaceNormal> &vTempSurfaceNormal,vector<FrameLine> &vVanishingDirection)
{

    cv::Mat R_cm_update=cv::Mat::zeros(cv::Size(3,3),CV_32F);
    vector<cv::Mat> vRotaionMatrix;
    vector<cv::Mat> vRotaionMatrix_good;
    cv::RNG rnger(cv::getTickCount());
    for(int k=0;k<1;k++)
    {
        //vector<cv::Point3d> vTempSurfaceNormal; //
        vector<cv::Mat> vSN_good;
        vector<double> lambda_good;
        vector<cv::Point2d> m_j_selected;
        //cv::Mat R_cm_update=cv::Mat::eye(cv::Size(3,3),CV_32F);


        //初始化 R——cm_update
        rnger.fill(R_cm_update, cv::RNG::UNIFORM, cv::Scalar::all(0.01), cv::Scalar::all(1));
        //cout<<"Random"<<R_cm_update<<endl;
        /* Eigen::Quaterniond qnorm;
         Eigen::Quaterniond q(R_cm_update.at<float>(0,0),R_cm_update.at<float>(0,1),R_cm_update.at<float>(0,2),R_cm_update.at<float>(1,0));//=Eigen::MatrixXd::Random(1, 4);
         cout<<"init q"<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w()<<endl;
         qnorm.x()=q.x()/q.norm();qnorm.y()=q.y()/q.norm();
         qnorm.z()=q.z()/q.norm();qnorm.w()=q.w()/q.norm();
         eigen2cv(qnorm.matrix(),R_cm_update);//	eigen2cv(m, img);;*/

        cv::SVD svd; cv::Mat U,W,VT;
        svd.compute(R_cm_update,W,U,VT);
        R_cm_update=U*VT;

        //cout<<"init"<<qnorm.x()<<","<<qnorm.y()<<","<<qnorm.z()<<","<<qnorm.w()<<","<<R_cm_update<<endl;

        //cout<<"init"<<R_cm_update<<endl;
        cv::Mat R_cm_Rec=cv::Mat::zeros(cv::Size(3,3),CV_32F);
        cv::Mat R_cm_initial;
        int  validMF=0;

        /*cout<<"Tracking,vSurfaceNormal"<<vSurfaceNormal->size()<<endl;
        //转化过来 surfacenomal
        for(size_t i=0;i<vSurfaceNormal->size();i++)
        {
            vTempSurfaceNormal.push_back(cv::Point3d(vSurfaceNormal->at(i).normal_x,vSurfaceNormal->at(i).normal_y,vSurfaceNormal->at(i).normal_z));
            cout<<"point"<<vSurfaceNormal->at(i).normal_x<<vSurfaceNormal->at(i).normal_y<<vSurfaceNormal->at(i).normal_z<<endl;
        }*/
        int numPlaneFound=0;

        //cout<<" vector<cv::Point3d> vTempSurfaceNormal"<<endl;
        //cv::Mat R=cv::Mat::eye(cv::Size(3,3),CV_32FC1);
        for(int i=0;i<10;i++)
        {

            cv::Mat R_cm=R_cm_update;//cv::Mat::eye(cv::Size(3,3),CV_32FC1);  // 对角线为1的对角矩阵(3, 3, CV_32FC1);
            //cout<<"R_cm"<<R_cm<<endl;
            int directionFound1=0;int directionFound2=0;int directionFound3=0; //三个方向
            int numDirectionFound=0;
            validMF=0;
            for(int a=1;a<4;a++)
            {
                cv::Mat ra =ProjectSN2MF( a,R_cm,vTempSurfaceNormal,vVanishingDirection);
                //cout<<"test projectSN2MF"<<ra<<endl;
                //如果 ra不为0
                if(sum(ra)[0]!=0)
                {
                    numDirectionFound += 1;
                    if(a==1) directionFound1=1;//确定发现a=1面
                    else if(a==2) directionFound2=1;
                    else if(a==3) directionFound3=1;
                    R_cm_Rec.at<float>(0,a-1) =ra.at<float>(0,0);
                    R_cm_Rec.at<float>(1,a-1) =ra.at<float>(1,0);
                    R_cm_Rec.at<float>(2,a-1) =ra.at<float>(2,0);
                }
            }
            //cout<<"R_cm_Rec"<<R_cm_Rec<<endl;
            //cout<<"direction:"<<directionFound1<<","<<directionFound2<<","<<directionFound3<<endl;
            if(numDirectionFound<2)
            {
                numDirectionFound=0;
                validMF=0;
                directionFound1=0;directionFound2=0;directionFound3=0;
                break;
            }
            else if(numDirectionFound==2)
            {
                if(directionFound1&&directionFound2)
                {
                    cv::Mat v1=R_cm_Rec.colRange(0,1).clone();
                    //cout<<1<<v1<<endl;
                    cv::Mat v2=R_cm_Rec.colRange(1,2).clone();
                    cv::Mat v3 = v1.cross(v2);
                    R_cm_Rec.at<float>(0,2) =v3.at<float>(0,0);
                    R_cm_Rec.at<float>(1,2) =v3.at<float>(1,0);
                    R_cm_Rec.at<float>(2,2) =v3.at<float>(2,0);
                    if(abs(cv::determinant(R_cm_Rec)+1)<0.5)
                    {
                        R_cm_Rec.at<float>(0,2) =-v3.at<float>(0,0);
                        R_cm_Rec.at<float>(1,2) =-v3.at<float>(1,0);
                        R_cm_Rec.at<float>(2,2) =-v3.at<float>(2,0);
                    }

                }else if(directionFound2&&directionFound3)
                {
                    cv::Mat v2=R_cm_Rec.colRange(1,2).clone();
                    cv::Mat v3=R_cm_Rec.colRange(2,3).clone();
                    cv::Mat v1 = v3.cross(v2);
                    R_cm_Rec.at<float>(0,0) =v1.at<float>(0,0);
                    R_cm_Rec.at<float>(1,0) =v1.at<float>(1,0);
                    R_cm_Rec.at<float>(2,0) =v1.at<float>(2,0);
                    if(abs(cv::determinant(R_cm_Rec)+1)<0.5)
                    {
                        R_cm_Rec.at<float>(0,0) =-v1.at<float>(0,0);
                        R_cm_Rec.at<float>(1,0) =-v1.at<float>(1,0);
                        R_cm_Rec.at<float>(2,0) =-v1.at<float>(2,0);
                    }
                }else if(directionFound1&&directionFound3)
                {
                    cv::Mat v1=R_cm_Rec.colRange(0,1).clone();
                    //cout<<"3-v3"<<v1<<endl;
                    cv::Mat v3=R_cm_Rec.colRange(2,3).clone();
                    ///cout<<"3-v3"<<v3<<endl;
                    cv::Mat v2 = v1.cross(v3);
                    R_cm_Rec.at<float>(0,1) =v2.at<float>(0,0);
                    R_cm_Rec.at<float>(1,1) =v2.at<float>(1,0);
                    R_cm_Rec.at<float>(2,1) =v2.at<float>(2,0);
                    if(abs(cv::determinant(R_cm_Rec)+1)<0.5)
                    {
                        R_cm_Rec.at<float>(0,1) =-v2.at<float>(0,0);
                        R_cm_Rec.at<float>(1,1) =-v2.at<float>(1,0);
                        R_cm_Rec.at<float>(2,1) =-v2.at<float>(2,0);
                    }

                }
            }

            //cout<<"R_cm_Rec_before"<<R_cm_Rec<<endl;
            numPlaneFound=numDirectionFound;
            //利用SVD分解
            cv::SVD svd; cv::Mat U,W,VT;
            svd.compute(R_cm_Rec,W,U,VT);
            R_cm_update=U*VT;
            //cout<<"R_cm_Rec_after"<<R_cm_update<<endl;
            validMF=1;
            //判断是否收敛
            if(acos((trace(R_cm.t()*R_cm_update)[0]-1.0))/2 < 0.01)
            {cout<<"trace "<<endl;break;}

        }

        //把candidate 装进rotationMatrix vector
        if(validMF==1)
        {
            vRotaionMatrix.push_back(R_cm_update);
        }


    }
    //check at least one manhattan frame
    size_t numMF= vRotaionMatrix.size();
    if(numMF==0)
    {
        cout<<"********not a Manhattan Frame**********"<<endl;
    }

    //移除 vRotationMatrix中多余的R
    vector<cv::Mat> vMF_CAN;
    vector<cv::Mat> vR_poss;
    int cellIndex=1;
    for (int row1=0;row1<3;row1++  )
        for(int row2=0;row2<3;row2++ )
            if(row2!=row1)
                for(int row3=0;row3<3;row3++ )
                {
                    if (row3 != row1 && row3 != row2)
                    {
                        cv::Mat R = cv::Mat::zeros(cv::Size(3,3),CV_32F);

                        R.at<float>(0,row1) = 1.0; R.at<float>(1,row2) = 1.0; R.at<float>(2,row3) = 1.0;
                        if (determinant(R)>0)
                        {
                            vR_poss.push_back(R);
                            cellIndex +=1;
                        }
                        R.at<float>(0,row1) = 1.0; R.at<float>(1,row2) = 1.0; R.at<float>(2,row3) = -1.0;
                        if (determinant(R)>0)
                        {
                            vR_poss.push_back(R);
                            cellIndex +=1;
                        }
                        R.at<float>(0,row1) = 1.0; R.at<float>(1,row2) = -1.0; R.at<float>(2,row3) = 1.0;
                        if (determinant(R)>0)
                        {
                            vR_poss.push_back(R);
                            cellIndex +=1;
                        }
                        R.at<float>(0,row1) = 1.0; R.at<float>(1,row2) = -1.0; R.at<float>(2,row3) = -1.0;
                        if (determinant(R)>0)
                        {
                            vR_poss.push_back(R);
                            cellIndex +=1;
                        }
                        R.at<float>(0,row1) = -1.0; R.at<float>(1,row2) = 1.0; R.at<float>(2,row3) = 1.0;
                        if (determinant(R)>0)
                        {
                            vR_poss.push_back(R);
                            cellIndex +=1;
                        }
                        R.at<float>(0,row1) = -1.0; R.at<float>(1,row2) = 1.0; R.at<float>(2,row3) = -1.0;
                        if (determinant(R)>0)
                        {
                            vR_poss.push_back(R);
                            cellIndex +=1;
                        }
                        R.at<float>(0,row1) = -1.0; R.at<float>(1,row2) = -1.0; R.at<float>(2,row3) = 1.0;
                        if (determinant(R)>0)
                        {
                            vR_poss.push_back(R);
                            cellIndex +=1;
                        }
                        R.at<float>(0,row1) = -1.0; R.at<float>(1,row2) = -1.0; R.at<float>(2,row3) = -1.0;
                        if (determinant(R)>0)
                        {
                            vR_poss.push_back(R);
                            cellIndex +=1;
                        }


                    }

                }
    // remove redundancy and convert to canonical coordinate
    size_t numRCandi=vRotaionMatrix.size();
    cout<<"rotation candidate"<<vRotaionMatrix.size()<<endl;
    for( size_t i=0;i<numRCandi;i++)
    {
        cv::Mat m_star;
        int minTheta=100;
        int minID=-1;
        for (int j=0;j<24;j++)
        {
            m_star=vRotaionMatrix[i]*vR_poss[j];
            double errTheta=acos((trace(m_star)[0]-1)/2);
            if(errTheta<minTheta)
            {
                minTheta=errTheta;
                minID=j;
            }

        }
        cout<<"good candidate"<<vRotaionMatrix[i]*vR_poss[minID]<<endl;
        vRotaionMatrix_good.push_back(vRotaionMatrix[i]*vR_poss[minID]);
    }
    //clean the null entries


    cout<<"VrOTATION GOOD"<<vRotaionMatrix_good.size()<<endl;
    double ratio=0.10;
    //cv::Mat R=ClusterMultiManhattanFrame(vRotaionMatrix_good,ratio);
    //cout<<"R"<<R<<endl;
    //做一个
    //

    /*for(int i=0;i<R_cm_update.rows;i++)
    {
        for(int j=0;j<R_cm_update.cols;j++)
            cout<<R_cm_update.at<float>(i,j)<<', ';
        cout<<endl;
    }*/
    return R_cm_update;//vRotaionMatrix_good[0];
}

cv::Mat Tracking::ClusterMultiManhattanFrame(vector<cv::Mat> &vRotationCandidate,double &clusterRatio)
{
    //MF_nonRd = [];
    vector<vector<int>> bin;
    //succ_rate = [];
    cv::Mat a;
    vector<cv::Mat> MF_nonRd;
    int histStart = 0;float histStep = 0.1;int histEnd = 2;
    int HasPeak = 1;
    int numMF_can = vRotationCandidate.size();
    int numMF = numMF_can;
    //rng(0,'twister');
    int numMF_nonRd = 0;

    while(HasPeak==1)
    {
        //随机的一个Rotation
        cv::Mat R=vRotationCandidate[rand() % (numMF_can-1)+ 1];
        cv::Mat tempAA;
        vector<cv::Point3f> Paa;
        //
        vector<float> vDistanceOfRotation;
        cv::Mat Rvec=R.t()*R;
        float theta=acos((trace(Rvec)[0]-1)/2);
        cv::Point3f w;
        w.x=theta*1/2*sin(theta)*(Rvec.at<float>(2,1)-Rvec.at<float>(1,2));
        w.y=theta*1/2*sin(theta)*(Rvec.at<float>(0,2)-Rvec.at<float>(2,0));
        w.z=theta*1/2*sin(theta)*(Rvec.at<float>(1,0)-Rvec.at<float>(0,1));

        for(int i=0;i<vRotationCandidate.size();i++)
        {
            cv::Mat RvecBetween=R.t()*vRotationCandidate[i];
            float theta=acos((trace(RvecBetween)[0]-1)/2);
            cv::Point3f wb;
            wb.x=theta*1/2*sin(theta)*(RvecBetween.at<float>(2,1)-RvecBetween.at<float>(1,2));
            wb.y=theta*1/2*sin(theta)*(RvecBetween.at<float>(0,2)-RvecBetween.at<float>(2,0));
            wb.z=theta*1/2*sin(theta)*(RvecBetween.at<float>(1,0)-RvecBetween.at<float>(0,1));
            Paa.push_back(wb);
            vDistanceOfRotation.push_back(norm(w-wb));
        }

        //
        bin=EasyHist(vDistanceOfRotation,histStart,histStep,histEnd);

        HasPeak = 0;
        for(int k = 0; k<bin.size();k++)
        {
            int binSize=0;
            for(int n=0;n<bin[k].size();n++){if(bin[k][n]>0)binSize++;}
            if (binSize/numMF_can > clusterRatio) {HasPeak=1;break;}
        }
        //if(HasPeak == 0) return;

        int binSize1=1;
        for(int n=0;n<bin[0].size();n++){if(bin[0][n]>0)binSize1++;}
        // check whether the dominant bin happens at zero
        if (binSize1/numMF_can > clusterRatio)
        {
            cv::Point3f meanPaaTem(0,0,0);
            for(int n=0;n<bin[0].size();n++)
            {
                if(bin[0][n]>0)
                {
                    meanPaaTem+=Paa[bin[0][n]];
                    meanPaaTem=meanPaaTem/binSize1;
                    meanPaaTem=meanPaaTem/norm(meanPaaTem);
                }

            }
            //calculate the mean
            float s = sin(norm(meanPaaTem));
            float c = cos(norm(meanPaaTem));
            float t = 1 - c;
            cv::Point3f vec_n(0,0,0);
            if (norm(meanPaaTem) <= 0.0001){}

            else
                vec_n = meanPaaTem;


            float x = vec_n.x;
            float y = vec_n.y;
            float z = vec_n.z;
            cv::Mat mm=cv::Mat::zeros(cv::Size(3,3),CV_32F);
            mm.at<float>(0,0)=t*x*x + c;mm.at<float>(0,1)= t*x*y - s*z;mm.at<float>(0,2)= t*x*z + s*y;
            mm.at<float>(1,0)=t*x*y + s*z; mm.at<float>(1,1)=t*y*y + c; mm.at<float>(1,2)= t*y*z - s*x;
            mm.at<float>(2,0)=t*x*z - s*y;mm.at<float>(2,1)= t*y*z + s*x;mm.at<float>(2,2)= t*z*z + c;

            if(isnan(sum(mm)[0])&&(norm(meanPaaTem)==0))
            {
                numMF_nonRd+=1;
                MF_nonRd.push_back(R);
            }
            else
            {
                numMF_nonRd = numMF_nonRd + 1;
                MF_nonRd.push_back(R * mm);
                //succ_rate{numMF_nonRd} = numel(bin{1})/numMF_can;
            }

            /*for(int j = 0;j<bin[0].size();j++)
            {
                if(bin[0][j]>0)
                {
                    vRotationCandidate[];
                }

            }*/
            //MF_can{bin{1}(j)} = [];

        }

    }
    return a;
}
vector<vector<int>> Tracking:: EasyHist(vector<float> &vDistance,int &histStart,float &histStep,int&histEnd)
{
    int numData = vDistance.size();
    int numBin = (histEnd - histEnd) / histStep;
    vector<vector<int>> bin(numBin, vector<int>(numBin, 0));//bin(numBin,0);
    for (int i=0;i<numBin;i++)
    {
        float down = (i-1)*histStep + histStart;
        float up = down + histStep;
        for (int j = 1;j<numData;j++)
        {
            if (vDistance[j] >= down && vDistance[j] < up)
                bin[i].push_back(j);//=bin[i]+1;
        }

    }
    return bin;

}
cv::Mat Tracking::ProjectSN2MF(int a,const cv::Mat &R_cm,const vector<SurfaceNormal> &vTempSurfaceNormal,vector<FrameLine> &vVanishingDirection)
{
    vector<cv::Point2d> m_j_selected;
    cv::Mat R_cm_Rec=cv::Mat::zeros(cv::Size(1,3),CV_32F);
    cv::Mat R_cm_NULL=cv::Mat::zeros(cv::Size(1,3),CV_32F);
    cv::Mat R_mc=cv::Mat::zeros(cv::Size(3,3),CV_32F);
    //R_Mc = [R_cM(:,mod(a+3,3)+1), R_cM(:,mod(a+4,3)+1), R_cM(:,mod(a+5,3)+1)].';

    int c1=(a+3)%3; int c2=(a+4)%3; int c3=(a+5)%3;
    R_mc.at<float>(0,0)=R_cm.at<float>(0,c1);R_mc.at<float>(0,1)=R_cm.at<float>(0,c2);
    R_mc.at<float>(0,2)=R_cm.at<float>(0,c3);R_mc.at<float>(1,0)=R_cm.at<float>(1,c1);
    R_mc.at<float>(1,1)=R_cm.at<float>(1,c2);R_mc.at<float>(1,2)=R_cm.at<float>(1,c3);
    R_mc.at<float>(2,0)=R_cm.at<float>(2,c1);R_mc.at<float>(2,1)=R_cm.at<float>(2,c2);
    R_mc.at<float>(2,2)=R_cm.at<float>(2,c3);
    R_mc=R_mc.t();




    //cout<<"R_cm"<<R_cm<<endl;

    /*cout<<"RCM"<<R_cm.at<float>(0,c1)<<", "<<R_cm.at<float>(1,c1)<<","<<R_cm.at<float>(2,c1)<<","<<
            R_cm.at<float>(0,c2)<<","<<R_cm.at<float>(1,c2)<<","<<R_cm.at<float>(2,c2)<<","<<
            R_cm.at<float>(0,c3)<<","<<R_cm.at<float>(1,c3)<<","<<R_cm.at<float>(2,c3)<<endl;*/
    size_t sizeOfSurfaceNormal=vTempSurfaceNormal.size()+vVanishingDirection.size();
    //cout<<"size of SN"<<sizeOfSurfaceNormal<<endl;
    for(size_t i=0;i<sizeOfSurfaceNormal;i++)
    {
        cv::Mat temp=cv::Mat::zeros(cv::Size(1,3),CV_32F);

        if(i>=vTempSurfaceNormal.size())
        {
            temp.at<float>(0,0)=vVanishingDirection[i].direction.x;
            temp.at<float>(1,0)=vVanishingDirection[i].direction.y;
            temp.at<float>(2,0)=vVanishingDirection[i].direction.z;
        }
        else
        {   temp.at<float>(0,0)=vTempSurfaceNormal[i].normal.x;
            temp.at<float>(1,0)=vTempSurfaceNormal[i].normal.y;
            temp.at<float>(2,0)=vTempSurfaceNormal[i].normal.z;
        }
        //cout<<temp<<endl;
        //cout<<" TEMP"<<vTempSurfaceNormal[i].x<<","<<vTempSurfaceNormal[i].y<<","<<vTempSurfaceNormal[i].z<<endl;

        cv::Point3f n_ini;
        cv::Mat m_ini;
        m_ini = R_mc * temp;
        n_ini.x=m_ini.at<float>(0,0);n_ini.y=m_ini.at<float>(1,0);n_ini.z=m_ini.at<float>(2,0);
        /*n_ini.x=R_mc.at<float>(0,0)*temp.at<float>(0,0)+R_mc.at<float>(0,1)*temp.at<float>(1,0)+R_mc.at<float>(0,2)*temp.at<float>(2,0);
        n_ini.y=R_mc.at<float>(1,0)*temp.at<float>(0,0)+R_mc.at<float>(1,1)*temp.at<float>(1,0)+R_mc.at<float>(1,2)*temp.at<float>(2,0);
        n_ini.z=R_mc.at<float>(2,0)*temp.at<float>(0,0)+R_mc.at<float>(2,1)*temp.at<float>(1,0)+R_mc.at<float>(2,2)*temp.at<float>(2,0);
        //cout<<"R_mc"<<R_mc<<endl;*/


        double lambda=sqrt(n_ini.x*n_ini.x+n_ini.y*n_ini.y);//at(k).y*n_a.at(k).y);
        //cout<<lambda<<endl;
        if(lambda<sin(0.2618)) //0.25
        {
            double tan_alfa=lambda/std::abs(n_ini.z);
            double alfa=asin(lambda);
            double m_j_x=alfa/tan_alfa*n_ini.x/n_ini.z;
            double m_j_y=alfa/tan_alfa*n_ini.y/n_ini.z;
            if(!std::isnan(m_j_x)&&!std::isnan(m_j_y))
                m_j_selected.push_back(cv::Point2d(m_j_x,m_j_y));
            if(i<vTempSurfaceNormal.size())
            {
                if(a==1)mCurrentFrame.vSurfaceNormalx.push_back(vTempSurfaceNormal[i].FramePosition);
                else if(a==2)mCurrentFrame.vSurfaceNormaly.push_back(vTempSurfaceNormal[i].FramePosition);
                else if(a==3)mCurrentFrame.vSurfaceNormalz.push_back(vTempSurfaceNormal[i].FramePosition);
            }
            else
            {
                if(a==1)
                {
                    cv::Point2d endPoint=vVanishingDirection[i].p;
                    cv::Point2d startPoint=vVanishingDirection[i].q;
                    vector<cv::Point2d> pointPair(2);
                    pointPair.push_back(endPoint);pointPair.push_back(startPoint);
                    mCurrentFrame.vVanishingLinex.push_back(pointPair);
                    for(int k=0;k<vVanishingDirection[i].rndpts3d.size();k++)
                        mCurrentFrame.vVaishingLinePCx.push_back(vVanishingDirection[i].rndpts3d[k]);
                }
                else if(a==2)
                {
                    cv::Point2d endPoint=vVanishingDirection[i].p;
                    cv::Point2d startPoint=vVanishingDirection[i].q;
                    vector<cv::Point2d> pointPair(2);
                    pointPair.push_back(endPoint);pointPair.push_back(startPoint);
                    mCurrentFrame.vVanishingLiney.push_back(pointPair);
                    for(int k=0;k<vVanishingDirection[i].rndpts3d.size();k++)
                        mCurrentFrame.vVaishingLinePCy.push_back(vVanishingDirection[i].rndpts3d[k]);
                }
                else if(a==3)
                {
                    cv::Point2d endPoint=vVanishingDirection[i].p;
                    cv::Point2d startPoint=vVanishingDirection[i].q;
                    vector<cv::Point2d> pointPair(2);
                    pointPair.push_back(endPoint);pointPair.push_back(startPoint);
                    mCurrentFrame.vVanishingLinez.push_back(pointPair);
                    for(int k=0;k<vVanishingDirection[i].rndpts3d.size();k++)
                        mCurrentFrame.vVaishingLinePCz.push_back(vVanishingDirection[i].rndpts3d[k]);
                }

            }
            //lambda_good.push_back(lambda);
            //找到一个面
        }
    }
    //cout<<"m_j_selected.push_back(temp)"<<m_j_selected.size()<<endl;

    if(m_j_selected.size()>sizeOfSurfaceNormal/20)
    {
        //cv::Point2d s_j = MeanShift(m_j_selected);
        sMS tempMeanShift = MeanShift(m_j_selected);
        cv::Point2d s_j = tempMeanShift.centerOfShift;// MeanShift(m_j_selected);
        float s_j_density=tempMeanShift.density;
        //cout<<"tracking:s_j"<<s_j.x<<","<<s_j.y<<endl;
        float alfa=norm(s_j);
        float ma_x=tan(alfa)/alfa*s_j.x;
        float ma_y=tan(alfa)/alfa*s_j.y;
        cv::Mat temp1=cv::Mat::zeros(cv::Size(1,3),CV_32F);
        temp1.at<float>(0,0)=ma_x;
        temp1.at<float>(1,0)=ma_y;
        temp1.at<float>(2,0)=1;

        R_cm_Rec=R_mc.t()*temp1;
        R_cm_Rec=R_cm_Rec/norm(R_cm_Rec); //列向量
        return R_cm_Rec;
    }

    return R_cm_NULL;

}
ResultOfMS Tracking::ProjectSN2MF(int a,const cv::Mat &R_mc,const vector<SurfaceNormal> &vTempSurfaceNormal,vector<FrameLine> &vVanishingDirection,const int numOfSN)
{
    vector<cv::Point2d> m_j_selected;
    cv::Mat R_cm_Rec=cv::Mat::zeros(cv::Size(1,3),CV_32F);
    cv::Mat R_cm_NULL=cv::Mat::zeros(cv::Size(1,3),CV_32F);
    ResultOfMS RandDen;
    RandDen.axis=a;

    size_t sizeOfSurfaceNormal=vTempSurfaceNormal.size()+vVanishingDirection.size();
    m_j_selected.reserve(sizeOfSurfaceNormal);

    for(size_t i=0;i<sizeOfSurfaceNormal;i++)
    {
        //cv::Mat temp=cv::Mat::zeros(cv::Size(1,3),CV_32F);

        cv::Point3f n_ini;
        int tepSize=i-vTempSurfaceNormal.size();
        if(i>=vTempSurfaceNormal.size())
        {


            n_ini.x = R_mc.at<float>(0,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(0,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(0,2) * vVanishingDirection[tepSize].direction.z;
            n_ini.y = R_mc.at<float>(1,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(1,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(1,2) * vVanishingDirection[tepSize].direction.z;
            n_ini.z = R_mc.at<float>(2,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(2,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(2,2) * vVanishingDirection[tepSize].direction.z;
        }
        else{

            n_ini.x = R_mc.at<float>(0,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(0,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(0,2) * vTempSurfaceNormal[i].normal.z;
            n_ini.y = R_mc.at<float>(1,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(1,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(1,2) * vTempSurfaceNormal[i].normal.z;
            n_ini.z = R_mc.at<float>(2,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(2,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(2,2) * vTempSurfaceNormal[i].normal.z;
        }


        double lambda=sqrt(n_ini.x*n_ini.x+n_ini.y*n_ini.y);//at(k).y*n_a.at(k).y);
        //cout<<lambda<<endl;
        //inside the cone
        if(lambda<sin(0.2518)) //0.25
        {
            double tan_alfa=lambda/std::abs(n_ini.z);
            double alfa=asin(lambda);
            double m_j_x=alfa/tan_alfa*n_ini.x/n_ini.z;
            double m_j_y=alfa/tan_alfa*n_ini.y/n_ini.z;
            if(!std::isnan(m_j_x)&&!std::isnan(m_j_y))
                m_j_selected.push_back(cv::Point2d(m_j_x,m_j_y));
            if(i<vTempSurfaceNormal.size())
            {
                if(a==1)
                {
                    mCurrentFrame.vSurfaceNormalx.push_back(vTempSurfaceNormal[i].FramePosition);
                    mCurrentFrame.vSurfacePointx.push_back(vTempSurfaceNormal[i].cameraPosition);
                }
                else if(a==2)
                {
                    mCurrentFrame.vSurfaceNormaly.push_back(vTempSurfaceNormal[i].FramePosition);
                    mCurrentFrame.vSurfacePointy.push_back(vTempSurfaceNormal[i].cameraPosition);
                }
                else if(a==3)
                {
                    mCurrentFrame.vSurfaceNormalz.push_back(vTempSurfaceNormal[i].FramePosition);
                    mCurrentFrame.vSurfacePointz.push_back(vTempSurfaceNormal[i].cameraPosition);
                }
            }
            else
            {
                if(a==1)
                {
                    cv::Point2d endPoint=vVanishingDirection[tepSize].p;
                    cv::Point2d startPoint=vVanishingDirection[tepSize].q;
                    vector<cv::Point2d> pointPair(2);
                    pointPair.push_back(endPoint);pointPair.push_back(startPoint);
                    mCurrentFrame.vVanishingLinex.push_back(pointPair);
                    for(int k=0;k<vVanishingDirection[tepSize].rndpts3d.size();k++)
                        mCurrentFrame.vVaishingLinePCx.push_back(vVanishingDirection[tepSize].rndpts3d[k]);
                }
                else if(a==2)
                {
                    cv::Point2d endPoint=vVanishingDirection[tepSize].p;
                    cv::Point2d startPoint=vVanishingDirection[tepSize].q;
                    vector<cv::Point2d> pointPair(2);
                    pointPair.push_back(endPoint);pointPair.push_back(startPoint);
                    mCurrentFrame.vVanishingLiney.push_back(pointPair);
                    for(int k=0;k<vVanishingDirection[tepSize].rndpts3d.size();k++)
                        mCurrentFrame.vVaishingLinePCy.push_back(vVanishingDirection[tepSize].rndpts3d[k]);
                }
                else if(a==3)
                {
                    cv::Point2d endPoint=vVanishingDirection[tepSize].p;
                    cv::Point2d startPoint=vVanishingDirection[tepSize].q;
                    vector<cv::Point2d> pointPair(2);
                    pointPair.push_back(endPoint);pointPair.push_back(startPoint);
                    mCurrentFrame.vVanishingLinez.push_back(pointPair);
                    for(int k=0;k<vVanishingDirection[tepSize].rndpts3d.size();k++)
                        mCurrentFrame.vVaishingLinePCz.push_back(vVanishingDirection[tepSize].rndpts3d[k]);
                }
            }


        }
    }
    //cout<<"a=1:"<<mCurrentFrame.vSurfaceNormalx.size()<<",a =2:"<<mCurrentFrame.vSurfaceNormaly.size()<<", a=3:"<<mCurrentFrame.vSurfaceNormalz.size()<<endl;
    //cout<<"m_j_selected.push_back(temp)"<<m_j_selected.size()<<endl;

    if(m_j_selected.size()>numOfSN)
    {
        sMS tempMeanShift = MeanShift(m_j_selected);
        cv::Point2d s_j = tempMeanShift.centerOfShift;// MeanShift(m_j_selected);
        float s_j_density=tempMeanShift.density;
        //cout<<"tracking:s_j"<<s_j.x<<","<<s_j.y<<endl;
        float alfa=norm(s_j);
        float ma_x=tan(alfa)/alfa*s_j.x;
        float ma_y=tan(alfa)/alfa*s_j.y;
        cv::Mat temp1=cv::Mat::zeros(cv::Size(1,3),CV_32F);
        temp1.at<float>(0,0)=ma_x;
        temp1.at<float>(1,0)=ma_y;
        temp1.at<float>(2,0)=1;
        cv::Mat rtemp=R_mc.t();
        R_cm_Rec=rtemp*temp1;
        R_cm_Rec=R_cm_Rec/norm(R_cm_Rec); //列向量
        RandDen.R_cm_Rec=R_cm_Rec;
        RandDen.s_j_density=s_j_density;

        return RandDen;
    }
    RandDen.R_cm_Rec=R_cm_NULL;
    return RandDen;

}

axiSNV Tracking::ProjectSN2Conic(int a,const cv::Mat &R_mc,const vector<SurfaceNormal> &vTempSurfaceNormal,vector<FrameLine> &vVanishingDirection)
{
    int numInConic=0;
    vector<cv::Point2d> m_j_selected;
    cv::Mat R_cm_Rec=cv::Mat::zeros(cv::Size(3,3),CV_32F);
    cv::Mat R_cm_NULL=cv::Mat::zeros(cv::Size(1,3),CV_32F);
    //cv::Mat R_mc=cv::Mat::zeros(cv::Size(3,3),CV_32F);
    vector<SurfaceNormal> vSNCadidate;
    axiSNV tempaxiSNV;
    tempaxiSNV.axis=a;


    size_t sizeOfSurfaceNormal=vTempSurfaceNormal.size();//TODO 先去掉线 +vVanishingDirection.size();
    tempaxiSNV.SNVector.reserve(sizeOfSurfaceNormal);
    //cout<<"size of SN"<<sizeOfSurfaceNormal<<endl;
    for(size_t i=0;i<sizeOfSurfaceNormal;i++)
    {

        cv::Point3f n_ini;
        if(i<vTempSurfaceNormal.size())
        {
            n_ini.x = R_mc.at<float>(0,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(0,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(0,2) * vTempSurfaceNormal[i].normal.z;
            n_ini.y = R_mc.at<float>(1,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(1,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(1,2) * vTempSurfaceNormal[i].normal.z;
            n_ini.z = R_mc.at<float>(2,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(2,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(2,2) * vTempSurfaceNormal[i].normal.z;

            double lambda=sqrt(n_ini.x*n_ini.x+n_ini.y*n_ini.y);//at(k).y*n_a.at(k).y);
            //cout<<lambda<<endl;
            if(lambda<sin(0.2518)) //0.25
            {

                //vSNCadidate.push_back(vTempSurfaceNormal[i]);
                //numInConic++;
                tempaxiSNV.SNVector.push_back(vTempSurfaceNormal[i]);



            }
        }
        else
        {   //cout<<"vanishing"<<endl;
            int tepSize=i-vTempSurfaceNormal.size();
            //cout<<vVanishingDirection[tepSize].direction.x<<"vanishing"<<endl;

            n_ini.x = R_mc.at<float>(0,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(0,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(0,2) * vVanishingDirection[tepSize].direction.z;
            n_ini.y = R_mc.at<float>(1,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(1,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(1,2) * vVanishingDirection[tepSize].direction.z;
            n_ini.z = R_mc.at<float>(2,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(2,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(2,2) * vVanishingDirection[tepSize].direction.z;

            double lambda=sqrt(n_ini.x*n_ini.x+n_ini.y*n_ini.y);//at(k).y*n_a.at(k).y);
            //cout<<lambda<<endl;
            if(lambda<sin(0.1518)) //0.25
            {

                //vSNCadidate.push_back(vTempSurfaceNormal[i]);
                //numInConic++;
                tempaxiSNV.Linesvector.push_back(vVanishingDirection[tepSize]);



            }

        }



    }

    return tempaxiSNV;//numInConic;

}

cv::Mat Tracking::TrackManhattanFrame(cv::Mat &mLastRcm,vector<SurfaceNormal> &vSurfaceNormal,vector<FrameLine> &vVanishingDirection)
{
    //上一帧的 camera 到 manhattan的距离
    //cout<<"begin Tracking Manhattan frame"<<endl;
    cv::Mat R_cm_update=mLastRcm.clone();
    //cout<<"mLastRcm"<<mLastRcm<<endl;
    int isTracked = 0;
    vector<double>denTemp(3, 0.00001);
    for (int i = 0; i < 6; i++) {

        cv::Mat R_cm = R_cm_update;//cv::Mat::eye(cv::Size(3,3),CV_32FC1);  // 对角线为1的对角矩阵(3, 3, CV_32FC1);
        //cout<<"R_cm"<<R_cm<<endl;
        int directionFound1 = 0;
        int directionFound2 = 0;
        int directionFound3 = 0; //三个方向
        int numDirectionFound = 0;
        vector<axiSNVector>vaxiSNV(4);
        vector<int> numInCone = vector<int>(3, 0);
        vector<cv::Point2f> vDensity ;
        //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for (int a = 1; a < 4; a++) {
            //在每个conic有多少 点
            cv::Mat R_mc=cv::Mat::zeros(cv::Size(3,3),CV_32F);
            int c1=(a+3)%3; int c2=(a+4)%3; int c3=(a+5)%3;
            R_mc.at<float>(0,0)=R_cm.at<float>(0,c1);R_mc.at<float>(0,1)=R_cm.at<float>(0,c2);
            R_mc.at<float>(0,2)=R_cm.at<float>(0,c3);R_mc.at<float>(1,0)=R_cm.at<float>(1,c1);
            R_mc.at<float>(1,1)=R_cm.at<float>(1,c2);R_mc.at<float>(1,2)=R_cm.at<float>(1,c3);
            R_mc.at<float>(2,0)=R_cm.at<float>(2,c1);R_mc.at<float>(2,1)=R_cm.at<float>(2,c2);
            R_mc.at<float>(2,2)=R_cm.at<float>(2,c3);
            cv::Mat R_mc_new=R_mc.t();
            //cout<<"R_mc_new"<<R_mc_new<<endl;
            vaxiSNV[a-1] = ProjectSN2Conic(a, R_mc_new, vSurfaceNormal,vVanishingDirection);
            numInCone[a - 1] = vaxiSNV[a-1].SNVector.size();
            //cout<<"2 a:"<<vaxiSNV[a-1].axis<<",vector:"<<numInCone[a - 1]<<endl;
        }
        //chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        //chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
        //cout << "first sN time: " << time_used.count() << endl;
        int minNumOfSN = vSurfaceNormal.size() / 20;
        //cout<<"minNumOfSN"<<minNumOfSN<<endl;
        //排序  a<b<c
        int a = numInCone[0];
        int b = numInCone[1];
        int c = numInCone[2];
        //cout<<"a:"<<a<<",b:"<<b<<",c:"<<c<<endl;
        int temp = 0;
        if (a > b) temp = a, a = b, b = temp;
        if (b > c) temp = b, b = c, c = temp;
        if (a > b) temp = a, a = b, b = temp;
        //cout<<"sequence  a:"<<a<<",b:"<<b<<",c:"<<c<<endl;
        //打印排序后的三个数
        if (b < minNumOfSN)
        {
            minNumOfSN = (b + a) / 2;
            //cout <<"thr"<<minNumOfSN<<endl;
        }

        //cout<<"new  minNumOfSN"<<minNumOfSN<<endl;
        //chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
        for (int a = 1; a < 4; a++)
        {
            cv::Mat R_mc=cv::Mat::zeros(cv::Size(3,3),CV_32F);
            int c1=(a+3)%3; int c2=(a+4)%3; int c3=(a+5)%3;
            R_mc.at<float>(0,0)=R_cm.at<float>(0,c1);R_mc.at<float>(0,1)=R_cm.at<float>(0,c2);
            R_mc.at<float>(0,2)=R_cm.at<float>(0,c3);R_mc.at<float>(1,0)=R_cm.at<float>(1,c1);
            R_mc.at<float>(1,1)=R_cm.at<float>(1,c2);R_mc.at<float>(1,2)=R_cm.at<float>(1,c3);
            R_mc.at<float>(2,0)=R_cm.at<float>(2,c1);R_mc.at<float>(2,1)=R_cm.at<float>(2,c2);
            R_mc.at<float>(2,2)=R_cm.at<float>(2,c3);
            cv::Mat R_mc_new=R_mc.t();
            vector<SurfaceNormal>* tempVVSN;
            vector<FrameLine>* tempLineDirection;
            for(int i=0;i<3;i++)
            {
                if(vaxiSNV[i].axis==a)
                {

                    tempVVSN=&vaxiSNV[i].SNVector;
                    tempLineDirection=&vaxiSNV[i].Linesvector;
                    //cout<<"2 a:"<<vaxiSNV[i].axis<<",vector:"<<tempVVSN.size()<<endl;
                    break;
                }

            }
            //cout<<"1 a:"<<a<<endl;

            ResultOfMS RD_temp  = ProjectSN2MF(a, R_mc_new, *tempVVSN,*tempLineDirection, minNumOfSN);

            //chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
            //chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t4-t3);
            //cout << "second SN time: " << time_used.count() << endl;

            //cout << "test projectSN2MF" << ra << endl;
            //如果 ra不为0
            if (sum(RD_temp.R_cm_Rec)[0] != 0) {
                numDirectionFound += 1;
                if (a == 1) directionFound1 = 1;//第一个轴
                else if (a == 2) directionFound2 = 1;
                else if (a == 3) directionFound3 = 1;
                R_cm_update.at<float>(0, a - 1) = RD_temp.R_cm_Rec.at<float>(0, 0);
                R_cm_update.at<float>(1, a - 1) = RD_temp.R_cm_Rec.at<float>(1, 0);
                R_cm_update.at<float>(2, a - 1) = RD_temp.R_cm_Rec.at<float>(2, 0);
                //RD_temp.s_j_density;

                vDensity.push_back(cv::Point2f(RD_temp.axis,RD_temp.s_j_density));

            }
        }
        //cout<<"numDirectionFound:"<<numDirectionFound<<endl;

        if (numDirectionFound < 2) {
            cout<<"oh, it has happened"<<endl;
            R_cm_update = R_cm;
            numDirectionFound = 0;
            isTracked = 0;
            directionFound1 = 0;
            directionFound2 = 0;
            directionFound3 = 0;
            break;
        } else if (numDirectionFound == 2) {
            if (directionFound1 && directionFound2) {
                cv::Mat v1 = R_cm_update.colRange(0, 1).clone();
                //cout << 1 << v1 << endl;
                cv::Mat v2 = R_cm_update.colRange(1, 2).clone();
                cv::Mat v3 = v1.cross(v2);
                R_cm_update.at<float>(0, 2) = v3.at<float>(0, 0);
                R_cm_update.at<float>(1, 2) = v3.at<float>(1, 0);
                R_cm_update.at<float>(2, 2) = v3.at<float>(2, 0);
                if (abs(cv::determinant(R_cm_update) + 1) < 0.5) {
                    R_cm_update.at<float>(0, 2) = -v3.at<float>(0, 0);
                    R_cm_update.at<float>(1, 2) = -v3.at<float>(1, 0);
                    R_cm_update.at<float>(2, 2) = -v3.at<float>(2, 0);
                }

            } else if (directionFound2 && directionFound3) {
                cv::Mat v2 = R_cm_update.colRange(1, 2).clone();
                //cout << 2 << v2 << endl;
                cv::Mat v3 = R_cm_update.colRange(2, 3).clone();
                cv::Mat v1 = v3.cross(v2);
                R_cm_update.at<float>(0, 0) = v1.at<float>(0, 0);
                R_cm_update.at<float>(1, 0) = v1.at<float>(1, 0);
                R_cm_update.at<float>(2, 0) = v1.at<float>(2, 0);
                if (abs(cv::determinant(R_cm_update) + 1) < 0.5) {
                    R_cm_update.at<float>(0, 0) = -v1.at<float>(0, 0);
                    R_cm_update.at<float>(1, 0) = -v1.at<float>(1, 0);
                    R_cm_update.at<float>(2, 0) = -v1.at<float>(2, 0);
                }
            } else if (directionFound1 && directionFound3) {
                cv::Mat v1 = R_cm_update.colRange(0, 1).clone();
                //cout << "3-v3" << v1 << endl;
                cv::Mat v3 = R_cm_update.colRange(2, 3).clone();
                //cout << "3-v3" << v3 << endl;
                cv::Mat v2 = v1.cross(v3);
                R_cm_update.at<float>(0, 1) = v2.at<float>(0, 0);
                R_cm_update.at<float>(1, 1) = v2.at<float>(1, 0);
                R_cm_update.at<float>(2, 1) = v2.at<float>(2, 0);
                if (abs(cv::determinant(R_cm_update) + 1) < 0.5) {
                    R_cm_update.at<float>(0, 1) = -v2.at<float>(0, 0);
                    R_cm_update.at<float>(1, 1) = -v2.at<float>(1, 0);
                    R_cm_update.at<float>(2, 1) = -v2.at<float>(2, 0);
                }

            }
        }

        //cout << "direction:" << directionFound1 << "," << directionFound2 << "," << directionFound3 << endl;
        //cout<<"svd before"<<R_cm_update<<endl;
        SVD svd;
        cv::Mat U, W, VT;
        /*for(int i=0;i<3;i++)
        {
            cout<<"v"<<vDensity[i].x<<endl;
            if(vDensity[i].x==1.0)
            {
                //cout<<"vy"<<vDensity[i].y<<endl;
                R_cm_update.at<float>(0,0)=vDensity[i].y* R_cm_update.at<float>(0,0);
                R_cm_update.at<float>(1,0)=vDensity[i].y* R_cm_update.at<float>(1,0);
                R_cm_update.at<float>(2,0)=vDensity[i].y* R_cm_update.at<float>(2,0);
            }
            if(vDensity[i].x==2.0)
            {
                R_cm_update.at<float>(0,1)=vDensity[i].y* R_cm_update.at<float>(0,1);
                R_cm_update.at<float>(1,1)=vDensity[i].y* R_cm_update.at<float>(1,1);
                R_cm_update.at<float>(2,1)=vDensity[i].y* R_cm_update.at<float>(2,1);
            }
            if(vDensity[i].x==3.0)
            {
                R_cm_update.at<float>(0,2)=vDensity[i].y* R_cm_update.at<float>(0,2);
                R_cm_update.at<float>(1,2)=vDensity[i].y* R_cm_update.at<float>(1,2);
                R_cm_update.at<float>(2,2)=vDensity[i].y* R_cm_update.at<float>(2,2);
            }
        }*/

        svd.compute(R_cm_update, W, U, VT);

        R_cm_update = U * VT;
        //cout<<"svd after"<<R_cm_update<<endl;

        //validMF=1;
        //判断是否收敛
        vDensity.clear();
        if (acos((trace(R_cm.t() * R_cm_update)[0] - 1.0)) / 2 < 0.001)
        {cout<<"go outside"<<endl;break;}
    }
    isTracked=1;
    return R_cm_update.clone();
}

sMS Tracking::MeanShift(vector<cv::Point2d> & v2D)
{
    sMS tempMS;
    int numPoint=v2D.size();
    float density;
    cv::Point2d nominator;
    double denominator=0;
    double nominator_x=0;
    double nominator_y=0;
    for(int i=0;i<numPoint;i++)
    {
        double k = exp(-20*norm(v2D.at(i))*norm(v2D.at(i)));
        nominator.x+=k*v2D.at(i).x;
        nominator.y+=k*v2D.at(i).y;
        denominator+=k;
    }
    tempMS.centerOfShift=nominator/denominator;
    tempMS.density=denominator/numPoint;

    return  tempMS;
}




void Tracking::StereoInitialization()
{
        if(mCurrentFrame.N>500  || mCurrentFrame.NL>15)
        {
            // Set Frame pose to the origin
            mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

            if(!mCurrentFrame.dealWithLine)
            {
                cout<<"Tracking, non-blur  image NL"<<mCurrentFrame.NL<<endl;
                if(mCurrentFrame.NL==0)
                {
                    cout<<"Tracking, extract lines for the current frame"<<endl;
                    LineSegment* mpLineSegment;
                    mpLineSegment->ExtractLineSegment(this->mImGray, mCurrentFrame.mvKeylinesUn, mCurrentFrame.mLdesc, mCurrentFrame.mvKeyLineFunctions);
                    mCurrentFrame.mvDepthLine = vector<float>(mCurrentFrame.mvKeylinesUn.size(),-1.0f);
                    mCurrentFrame.mvLines3D=vector<Vector6d>(mCurrentFrame.mvKeylinesUn.size(), static_cast<Vector6d>(NULL));
                    for(int i=0; i<mCurrentFrame.mvKeylinesUn.size(); ++i)	{ // each line
                        double len = cv::norm(mCurrentFrame.mvKeylinesUn[i].getStartPoint() - mCurrentFrame.mvKeylinesUn[i].getEndPoint());
                        vector<cv::Point3d> pts3d;
                        double numSmp = (double) min((int)len, 100); //number of line points sampled

                        //cout<<"numSmp:"<<numSmp<<endl;

                        pts3d.reserve(numSmp);//预留数量

                        for(int j=0; j<=numSmp; ++j) {
                            // use nearest neighbor to querry depth value
                            // assuming position (0,0) is the top-left corner of image, then the
                            // top-left pixel's center would be (0.5,0.5)
                            cv::Point2d pt = mCurrentFrame.mvKeylinesUn[i].getStartPoint() * (1-j/numSmp) + mCurrentFrame.mvKeylinesUn[i].getEndPoint()* (j/numSmp);

                            //cout<<"pt"<<pt.x<<","<<pt.y<<endl;
                            if(pt.x<0 || pt.y<0 || pt.x >= this->mImDepth.cols || pt.y >= this->mImDepth.rows ) continue;
                            int row, col; // nearest pixel for pt
                            if((floor(pt.x) == pt.x) && (floor(pt.y) == pt.y)) {// boundary issue
                                col = max(int(pt.x-1),0);
                                row = max(int(pt.y-1),0);
                            } else {
                                col = int(pt.x);
                                row = int(pt.y);
                            }
                            //cout<<"col:"<<col<<","<<"row:"<<row<<endl;
                            float d=-1;
                            if(this->mImGray.at<float>(row,col) <=0.01) { // no depth info
                                //cout<<"no depth info"<<endl;
                                continue;
                            }
                            else {
                                d = this->mImGray.ptr<float>(row)[col];
                            }
                            cv::Point3d p;

                            //cout<<"col:"<<col<<", row:"<<row<<endl;
                            //cout<<"cx:"<<cx<<",fx"<<fx<<endl;
                            // 计算这个点的空间坐标
                            p.z = d;
                            p.x = (col - mCurrentFrame.cx) * p.z *mCurrentFrame.invfx;//K.at<float>(0,0);//invfx;
                            p.y = (row - mCurrentFrame.cy) * p.z *mCurrentFrame.invfy;//K.at<float>(1,1);//invfy;
                            //cout<<"x:"<<p.x<<",y:"<<p.y<<",z:"<<p.z<<endl;

                            //mv3DLineforMap.push_back(p);
                            //origPoints_xyzrgb<<p.x<<" "<<p.y<<" "<<p.z<<endl;
                            pts3d.push_back(p);

                        }
                        //如果 点数量少于 10或者 ，就抛弃该线
                        if (pts3d.size() < 10.0)//sysPara.ratio_of_collinear_pts
                            continue;

                        RandomLine3d tmpLine;
#if 1
                        vector<RandomPoint3d> rndpts3d;
                        rndpts3d.reserve(pts3d.size());
                        //cout<<"pts3d size:"<<pts3d.size()<<endl;
                        // compute uncertainty of 3d points
                        for(int j=0; j<pts3d.size();++j) {
                            //cout<<"ptsxyz"<<pts3d[j].x<<","<<pts3d[j].y<<","<<pts3d[j].z<<endl;
                            rndpts3d.push_back(compPt3dCov(pts3d[j], mCurrentFrame.mK, 1));
                            //cout<<"Du[0]:"<<compPt3dCov(pts3d[j], mK, 1).DU[0]<<endl;
                        }
                        // using ransac to extract a 3d line from 3d pts
                        tmpLine = extract3dline_mahdist(rndpts3d);
#else
                        //计算这个线的参数
        tmpLine = extract3dline(pts3d,origPoints,optiPoints);
#endif
                        //cout<<"jieshu"<<endl;
                        //修复这条直线上的点


                        if(tmpLine.pts.size()/len > 0.4/*sysPara.ratio_of_collinear_pts*/	&&
                           cv::norm(tmpLine.A - tmpLine.B) >0.02) { // sysPara.line3d_length_thresh
                            //this line is reliable
                            mCurrentFrame.mvDepthLine[i]= 1.0f;
                            cout<<"gaibian line 3d"<<endl;
                            mCurrentFrame.mvLines3D[i]<<tmpLine.A.x,tmpLine.A.y,tmpLine.A.z,tmpLine.B.x,tmpLine.B.y,tmpLine.B.z;
                        }

                        /*if(1==mCurrentFrame.mvDepthLine[i])
                        {
                            // rebuild this line in depth map

                            for(int j=0; j<=numSmp; ++j)
                            {
                                cv::Point2d pt = mCurrentFrame.mvKeylinesUn[i].getStartPoint() * (1-j/numSmp) + mCurrentFrame.mvKeylinesUn[i].getEndPoint() * (j/numSmp);
                                if(pt.x<0 || pt.y<0 || pt.x >= this->mImGray.cols || pt.y >= this->mImGray.rows ) continue;

                                int row, col; // nearest pixel for pt
                                if((floor(pt.x) == pt.x) && (floor(pt.y) == pt.y)) {// boundary issue
                                    col = max(int(pt.x-1),0);
                                    row = max(int(pt.y-1),0);
                                }
                                else {
                                    col = int(pt.x);
                                    row = int(pt.y);
                                }


                                float d=this->mImGray.ptr<float>(row)[col];
                                cv::Point3d p;
                                if(d<=0.01) continue;
                                else
                                {
                                    p.z=d;
                                    if (p.z < 0.01 || p.z>10){cout<<"Frame: invalid p.z"<<endl;continue;}

                                    p.x = (col - mCurrentFrame.cx) * p.z *mCurrentFrame.invfx;
                                    p.y = (row - mCurrentFrame.cy) * p.z *mCurrentFrame.invfy;
                                    p.z=mCurrentFrame.fy*mCurrentFrame.fx*(tmpLine.director.y*tmpLine.mid.x-
                                               tmpLine.director.x*tmpLine.mid.y)/
                                        (mCurrentFrame.fy*tmpLine.director.y*(col-mCurrentFrame.cx)-
                                                mCurrentFrame.fx*tmpLine.director.x*(row-mCurrentFrame.cy));
                                    //cout<<p.x<<", "<<p.y<<", "<<p.z<<endl;
                                    mCurrentFrame.mv3DLineforMap.push_back(p);
                                }
                                //tmpLine.pts.push_back(p);


                            }
                        }*/
                    }

                    mCurrentFrame.NL=mCurrentFrame.mvKeylinesUn.size();
                    mCurrentFrame.mvpMapLines=vector<MapLine *>(mCurrentFrame.NL,static_cast<MapLine*>(NULL));
                    mCurrentFrame.mvbLineOutlier=vector<bool>(mCurrentFrame.NL,false);
                    cout<<"Tracking, extract lines from non-blur KF,NL"<<mCurrentFrame.NL<<endl;

                }

            }
            // Create KeyFrame
            KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

            // Insert KeyFrame in the map
            mpMap->AddKeyFrame(pKFini);

            // Create MapPoints and asscoiate to KeyFrame
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                    pNewMP->AddObservation(pKFini,i);
                    pKFini->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);
                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                }
            }


            for(int i=0; i<mCurrentFrame.NL;i++)
            {
                //检查 线的深度是否存在

                float isDepthGood= mCurrentFrame.mvDepthLine[i];
                //cout<<"Tracking: StereoInitialization:线是否存在:"<<isDepthGood<<endl;

                //如果存在， 执行程序
                if(isDepthGood==1)
                {
                    //cout<<"i="<<i<<",the size of mvLines3D"<<mCurrentFrame.mvLines3D.size()<<endl;
                    //cout<<"Tracking:mCurrentFrame.mvLines3D[i];"<<mCurrentFrame.mvLines3D[i]<<endl;
                    Vector6d line3D= mCurrentFrame.obtain3DLine(i);//mvLines3D[i];
                    MapLine *pNewML=new MapLine(line3D,pKFini,mpMap);
                    pNewML->AddObservation(pKFini,i);
                    pKFini->AddMapLine(pNewML,i);
                    pNewML->ComputeDistinctiveDescriptors();
                    pNewML->UpdateAverageDir();
                    mpMap->AddMapLine(pNewML);
                    mCurrentFrame.mvpMapLines[i]=pNewML;
                }
            }

            cout << "Tracking: inital: New map created with " << mpMap->MapPointsInMap() << " points" << endl;
            cout << "Tracking: inital: New map created with " << mpMap->MapLinesInMap() << " Lines" << endl;

            mpLocalMapper->InsertKeyFrame(pKFini);

            mLastFrame = Frame(mCurrentFrame);
            mnLastKeyFrameId=mCurrentFrame.mnId;
            mpLastKeyFrame = pKFini;

            mvpLocalKeyFrames.push_back(pKFini);
            mvpLocalMapPoints=mpMap->GetAllMapPoints();
            mvpLocalMapLines=mpMap->GetAllMapLines();

            mpReferenceKF = pKFini;
            mCurrentFrame.mpReferenceKF = pKFini;

            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
            mpMap->SetReferenceMapLines(mvpLocalMapLines);

            mpMap->mvpKeyFrameOrigins.push_back(pKFini);

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            mState=OK;
        }
}

void Tracking::MonocularInitialization()
{
        int num = 50;
        // 如果单目初始器还没有没创建，则创建单目初始器
        if(!mpInitializer)
        {
            // Set Reference Frame
            if(mCurrentFrame.mvKeys.size()>num)
            {
                // step 1：得到用于初始化的第一帧，初始化需要两帧
                mInitialFrame = Frame(mCurrentFrame);
                // 记录最近的一帧
                mLastFrame = Frame(mCurrentFrame);
                // mvbPreMatched最大的情况就是当前帧所有的特征点都被匹配上
                mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
                for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

                if(mpInitializer)
                    delete mpInitializer;

                // 由当前帧构造初始化器， sigma:1.0    iterations:200
                mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

                fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

                return;
            }
        }
        else
        {
            // Try to initialize
            // step2：如果当前帧特征点数大于100，则得到用于单目初始化的第二帧
            // 如果当前帧特征点太少，重新构造初始器
            // 因此只有连续两帧的特征点个数都大于100时，才能继续进行初始化过程
            if((int)mCurrentFrame.mvKeys.size()<=num)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer*>(NULL);
                fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
                return;
            }

            // Find correspondences
            // step3：在mInitialFrame与mCurrentFrame中找匹配的特征点对
            // mvbPrevMatched为前一帧的特征点，存储了mInitialFrame中哪些点将进行接下来的匹配,类型  std::vector<cv::Point2f> mvbPrevMatched;
            // mvIniMatches存储mInitialFrame, mCurrentFrame之间匹配的特征点，类型为std::vector<int> mvIniMatches; ????
            ORBmatcher matcher(0.9,true);
            int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

            LSDmatcher lmatcher;   //建立线特征之间的匹配
            int lineMatches = lmatcher.SerachForInitialize(mInitialFrame, mCurrentFrame, mvLineMatches);
//        cout << "Tracking::MonocularInitialization(), lineMatches = " << lineMatches << endl;
//
//        cout << "Tracking::MonocularInitialization(), mvLineMatches size = " << mvLineMatches.size() << endl;

            // Check if there are enough correspondences
            // step4：如果初始化的两帧之间的匹配点太少，重新初始化
            if(nmatches<30)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer*>(NULL);
                return;
            }

            cv::Mat Rcw; // Current Camera Rotation
            cv::Mat tcw; // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

            // step5：通过H或者F进行单目初始化，得到两帧之间相对运动，初始化MapPoints
#if 0
            if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            // 将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            // step6：将三角化得到的3D点包装成MapPoints
            /// 如果要修改，应该是从这个函数开始
            CreateInitialMapMonocular();
        }
#else
            if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated, mvLineMatches, mvLineS3D, mvLineE3D, mvbLineTriangulated))//mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated, mvLineMatches, mvLineS3D, mvLineE3D, mvbLineTriangulated))
            {
                for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
                {
                    if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                    {
                        mvIniMatches[i]=-1;
                        nmatches--;
                    }
                }

                // Set Frame Poses
                // 将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
                mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(Tcw.rowRange(0,3).col(3));
                mCurrentFrame.SetPose(Tcw);

                // step6：将三角化得到的3D点包装成MapPoints
                /// 如果要修改，应该是从这个函数开始
                //CreateInitialMapMonocular();
                CreateInitialMapMonoWithLine();
            }
#endif
        }
    }

/**
 * @brief 为单目摄像头三角化生成MapPoints，只有特征点
 */
void Tracking::CreateInitialMapMonocular()
{
        // Create KeyFrames
        KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
        KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // step 1: 将初始关键帧的描述子转为BoW
        pKFini->ComputeBoW();
        // step 2：将当前关键帧的描述子转为BoW
        pKFcur->ComputeBoW();

        // Insert KFs in the map
        // step3：将关键帧插入到地图，凡是关键帧，都要插入地图
        mpMap->AddKeyFrame(pKFini);
        mpMap->AddKeyFrame(pKFcur);

        // Create MapPoints and asscoiate to keyframes
        // step4：将特征点的3D点包装成MapPoints
        for(size_t i=0; i<mvIniMatches.size();i++)
        {
            if(mvIniMatches[i]<0)
                continue;

            //Create MapPoint.
            cv::Mat worldPos(mvIniP3D[i]);

            // step4.1：用3D点构造MapPoint
            MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

            // step4.2：为该MapPoint添加属性：
            // a.观测到该MapPoint的关键帧
            // b.该MapPoint的描述子
            // c.该MapPoint的平均观测方向和深度范围

            // step4.3：表示该KeyFrame的哪个特征点可以观测到哪个3D点
            pKFini->AddMapPoint(pMP,i);
            pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

            // a.表示该MapPoint可以被哪个KeyFrame观测到，以及对应的第几个特征点
            pMP->AddObservation(pKFini,i);
            pMP->AddObservation(pKFcur,mvIniMatches[i]);

            // b.从众多观测到该MapPoint的特征点中挑选出区分度最高的描述子
            pMP->ComputeDistinctiveDescriptors();

            // c.更新该MapPoint的平均观测方向以及观测距离的范围
            pMP->UpdateNormalAndDepth();

            //Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            //Add to Map
            // step4.4：在地图中添加该MapPoint
            mpMap->AddMapPoint(pMP);
        }

        // Update Connections
        // step5:更新关键帧间的连接关系
        // 在3D点和关键帧之间建立边，每一个边有一个权重，边的权重是该关键帧与当前关键帧公共3D点的个数
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        // Bundle Adjustment
        cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

        // step6：BA优化
        Optimizer::GlobalBundleAdjustemnt(mpMap,20);

        // Set median depth to 1
        // step7：将MapPoints的中值深度归一化到1，并归一化两帧之间的变换
        // 评估关键帧场景深度，q=2表示中值
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth = 1.0f/medianDepth;

        if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
        {
            cout << "Wrong initialization, reseting..." << endl;
            Reset();
            return;
        }

        // Scale initial baseline
        cv::Mat Tc2w = pKFcur->GetPose();
        Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale points
        vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
        for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
        {
            if(vpAllMapPoints[iMP])
            {
                MapPoint* pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            }
        }

        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        mLastFrame = Frame(mCurrentFrame);

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mState=OK;  //至此，初始化成功
    }






#if 1
/**
 * @brief 为单目摄像头三角化生成带有线特征的Map，包括MapPoints和MapLine
 */
void Tracking::CreateInitialMapMonoWithLine()
{
        // step1:创建关键帧，即用于初始化的前两帧
        KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
        KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

        // step2：将两个关键帧的描述子转为BoW，这里的BoW只有ORB的词袋
        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // step3：将关键帧插入到地图，凡是关键帧，都要插入地图
        mpMap->AddKeyFrame(pKFini);
        mpMap->AddKeyFrame(pKFcur);

        // step4：将特征点的3D点包装成MapPoints
        for(size_t i=0; i<mvIniMatches.size(); i++)
        {
            if(mvIniMatches[i]<0)
                continue;

            // Create MapPoint
            cv::Mat worldPos(mvIniP3D[i]);

            // step4.1：用3D点构造MapPoint
            MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);

            // step4.2：为该MapPoint添加属性：
            // a.观测到该MapPoint的关键帧
            // b.该MapPoint的描述子
            // c.该MapPoint的平均观测方向和深度范围

            // step4.3：表示该KeyFrame的哪个特征点对应到哪个3D点
            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

            // a.表示该MapPoint可以被哪个KeyFrame观测到，以及对应的第几个特征点
            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, mvIniMatches[i]);

            // b.从众多观测到该MapPoint的特征点中挑选出区分度最高的描述子
            pMP->UpdateNormalAndDepth();

            // Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            // Add to Map
            // step4.4：在地图中添加该MapPoint
            mpMap->AddMapPoint(pMP);
        }

        // step5：将特征线包装成MapLines
        for(size_t i=0; i<mvLineMatches.size(); i++)
        {
            if(!mvbLineTriangulated[i])
                continue;

            // Create MapLine
            Vector6d worldPos;
            worldPos << mvLineS3D[i].x, mvLineS3D[i].y, mvLineS3D[i].z, mvLineE3D[i].x, mvLineE3D[i].y, mvLineE3D[i].z;

            //step5.1：用线段的两个端点构造MapLine
            MapLine* pML = new MapLine(worldPos, pKFcur, mpMap);

            //step5.2：为该MapLine添加属性：
            // a.观测到该MapLine的关键帧
            // b.该MapLine的描述子
            // c.该MapLine的平均观测方向和深度范围？

            //step5.3：表示该KeyFrame的哪个特征点可以观测到哪个3D点
            pKFini->AddMapLine(pML,i);
            pKFcur->AddMapLine(pML,i);

            //a.表示该MapLine可以被哪个KeyFrame观测到，以及对应的第几个特征线
            pML->AddObservation(pKFini, i);
            pML->AddObservation(pKFcur, i);

            //b.MapPoint中是选取区分度最高的描述子，pl-slam直接采用前一帧的描述子,这里先按照ORB-SLAM的过程来
            pML->ComputeDistinctiveDescriptors();

            //c.更新该MapLine的平均观测方向以及观测距离的范围
            pML->UpdateAverageDir();

            // Fill Current Frame structure
            mCurrentFrame.mvpMapLines[i] = pML;
            mCurrentFrame.mvbLineOutlier[i] = false;

            // step5.4: Add to Map
            mpMap->AddMapLine(pML);
        }

        // step6：更新关键帧间的连接关系
        // 1.最初是在3D点和关键帧之间建立边，每一个边有一个权重，边的权重是该关键帧与当前关键帧公共3D点的个数
        // 2.加入线特征后，这个关系应该和特征线也有一定的关系，或者就先不加关系，只是单纯的添加线特征

        // step7：全局BA优化，这里需要再进一步修改优化函数，参照OptimizePose函数
        cout << "this Map created with " << mpMap->MapPointsInMap() << " points, and "<< mpMap->MapLinesInMap() << " lines." << endl;
        //Optimizer::GlobalBundleAdjustemnt(mpMap, 20, true); //true代表使用有线特征的BA

        // step8：将MapPoints的中值深度归一化到1，并归一化两帧之间的变换
        // Q：MapPoints的中值深度归一化为1，MapLine是否也归一化？
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth = 1.0f/medianDepth;

        cout << "medianDepth = " << medianDepth << endl;
        cout << "pKFcur->TrackedMapPoints(1) = " << pKFcur->TrackedMapPoints(1) << endl;

        if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<80)
        {
            cout << "Wrong initialization, reseting ... " << endl;
            Reset();
            return;
        }

        // Scale initial baseline
        cv::Mat Tc2w = pKFcur->GetPose();
        Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale Points
        vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); ++iMP)
        {
            if(vpAllMapPoints[iMP])
            {
                MapPoint* pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            }
        }

        // Scale Line Segments
        vector<MapLine*> vpAllMapLines = pKFini->GetMapLineMatches();
        for(size_t iML=0; iML < vpAllMapLines.size(); iML++)
        {
            if(vpAllMapLines[iML])
            {
                MapLine* pML = vpAllMapLines[iML];
                pML->SetWorldPos(pML->GetWorldPos()*invMedianDepth);
            }
        }

        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mvpLocalMapLines = mpMap->GetAllMapLines();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        mLastFrame = Frame(mCurrentFrame);

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
        mpMap->SetReferenceMapLines(mvpLocalMapLines);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mState = OK;
    }

#endif

void Tracking::CheckReplacedInLastFrame()
{
        // 点特征
        for(int i =0; i<mLastFrame.N; i++)
        {
            MapPoint* pMP = mLastFrame.mvpMapPoints[i];

            if(pMP)
            {
                MapPoint* pRep = pMP->GetReplaced();
                if(pRep)
                {
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
        // 线特征

        for(int i=0; i<mLastFrame.NL; i++)
        {
            MapLine* pML = mLastFrame.mvpMapLines[i];

            if(pML)
            {
                MapLine* pReL = pML->GetReplaced();
                if(pReL)
                {
                    mLastFrame.mvpMapLines[i] = pReL;
                }
            }
        }
    }


bool Tracking::TrackReferenceKeyFrame()
{
        //cout<<"Tracking: with reference keyframe"<<endl;
        // Compute Bag of Words vector
        mCurrentFrame.ComputeBoW();
        int lmatches=0;
        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.9,true);
        LSDmatcher lmatcher;

        vector<MapPoint*> vpMapPointMatches;
        vector<MapLine*> vpMapLineMatches;
        vector<pair<int, int>> vLineMatches;

        int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
        cout<<"Tracking REFERENCE mmatches:"<<nmatches<<endl;

        if(mCurrentFrame.dealWithLine)
        {   lmatches = lmatcher.SearchByProjection(mpReferenceKF,mCurrentFrame,vpMapLineMatches);
            mCurrentFrame.mvpMapLines=vpMapLineMatches;
            cout<<"Tracking REFERENCE lmatches:"<<lmatches<<endl;
//            if(nmatches<15 &&lmatches<5)
//                return false;
        }
        else
        {
            if(nmatches<15)
                return false;
        }


        mCurrentFrame.mvpMapPoints = vpMapPointMatches;

        for(size_t i=0;i<mCurrentFrame.mvpMapLines.size();i++)
        {
            MapLine * ml=mCurrentFrame.mvpMapLines[i];
            if(ml)
            {
                Eigen::Vector3d line_obs=mCurrentFrame.mvKeyLineFunctions[i];
                //cout<<"TRACKING i,line_obes"<<i<<","<<line_obs[0]<<","<<line_obs[1]<<","<<line_obs[2]<<endl;
                //cout<<"tracking:"<<ml->mWorldPos[0]<<","<<ml->mWorldPos[1]<<","<<ml->mWorldPos[2]<<endl;
            }
        }
        //mCurrentFrame.mvpMapLines=
        mCurrentFrame.SetPose(mLastFrame.mTcw);
        cout<<"tracking reference,pose before opti"<<mLastFrame.mTcw<<endl;
        // 通过优化3D-2D的重投影误差来获得位姿
        if(mCurrentFrame.dealWithLine)
            Optimizer::PoseOptimization(&mCurrentFrame, true);
        else
            Optimizer::PoseOptimization(&mCurrentFrame);
        cout<<"tracking reference,pose after opti"<<mCurrentFrame.mTcw<<endl;
        // Discard outliers
        // 剔除优化后的outlier匹配点（MapPoints）
        int nmatchesMap = 0;
        int nmatchesLineMap=0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;

                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }


        //当 cMurrentFrame.dealWithLine为true，使用Pose去除错误的线
        for(int i=0;i<mCurrentFrame.NL;i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                if(mCurrentFrame.mvbLineOutlier[i])
                {
                    MapLine* pML=mCurrentFrame.mvpMapLines[i];
                    mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
                    mCurrentFrame.mvbLineOutlier[i]=false;
                    pML->mbTrackInView=false;
                    pML->mnLastFrameSeen=mCurrentFrame.mnId;
                    lmatches--;
                    cout<<"tracking,lmatches"<<lmatches<<endl;
                }
                else if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                    nmatchesLineMap++;

            }
        }

        if(mCurrentFrame.dealWithLine)
            return  nmatchesMap>=10||nmatchesLineMap>=5;
        else
            return nmatchesMap>=10;
}


/**
 * @brief 双目或rgbd相机根据深度值为上一帧产生新的MapPoints
 *
 * 在双目和rgbd情况下，选取一些深度小一些的点（可靠一些）
 * 可以通过深度值产生一些新的MapPoints
 */
void Tracking::UpdateLastFrame()
{
        // Update pose according to reference keyframe
        // step1: 更新最近一帧的位姿
        KeyFrame* pRef = mLastFrame.mpReferenceKF;
        cv::Mat Tlr = mlRelativeFramePoses.back();

        mLastFrame.SetPose(Tlr*pRef->GetPose());

        // 如果上一帧为关键帧，或者单目的情况，则退出
        if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
            return;

        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mLastFrame.N);
        for(int i=0; i<mLastFrame.N;i++)
        {
            float z = mLastFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        //if(vDepthIdx.empty())
        //    return;

        sort(vDepthIdx.begin(),vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for(size_t j=0; j<vDepthIdx.size();j++)
        {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            MapPoint* pMP = mLastFrame.mvpMapPoints[i];
            if(!pMP)
                bCreateNew = true;
            else if(pMP->Observations()<1)
            {
                bCreateNew = true;
            }

            if(bCreateNew)
            {
                cv::Mat x3D = mLastFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

                mLastFrame.mvpMapPoints[i]=pNewMP;

                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            }
            else
            {
                nPoints++;
            }

            if(vDepthIdx[j].first>mThDepth && nPoints>100)
                break;
        }


        // Create "visual odometry" MapLines
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float,int> > vLineDepthIdx;
        vLineDepthIdx.reserve(mLastFrame.NL);
        int nLines = 0;
        for(int i=0; i<mLastFrame.NL;i++)
        {
            float z = mLastFrame.mvDepthLine[i];
            if(z==1)
            {
                bool bCreateNew = false;
                vLineDepthIdx.push_back(make_pair(z,i));
                MapLine *pML = mLastFrame.mvpMapLines[i];
                if (!pML)
                    bCreateNew = true;
                else if (pML->Observations() < 1) {
                    bCreateNew = true;
                }
                if (bCreateNew) {
                    Vector6d line3D= mLastFrame.obtain3DLine(i);//mvLines3D[i];
                    MapLine *pNewML=new MapLine(line3D,mpMap,&mLastFrame,i);
                    //Vector6d x3D = mLastFrame.mvLines3D(i);
                    //MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

                    mLastFrame.mvpMapLines[i] = pNewML;

                    mlpTemporalLines.push_back(pNewML);
                    nLines++;
                } else {
                    nLines++;
                }

                if ( nLines> 30)
                    break;

            }
        }


}


/**
 * @brief 根据匀速模型对上一帧的MapPoints进行跟踪
 *
 * 1.非单目情况，需要对上一帧产生一些新的MapPoints(临时)
 * 2.将上一帧的MapPoints投影到当前帧的图像平面上，在投影的位置进行区域匹配
 * 3.根据匹配对估计当前帧的姿态
 * 4.根据姿态剔除误匹配
 * @return 如果匹配数大于10，则返回true
 */
bool Tracking::TrackWithMotionModel()
{
        cout<<"Tracking: with motion model"<<endl;
        // --step1: 建立ORB特征点的匹配
        ORBmatcher matcher(0.9,true);
        LSDmatcher lmatcher;

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        // --step2: 更新上一帧的位姿
        UpdateLastFrame();

        // --step3:根据Const Velocity Model，估计当前帧的位姿
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        // Project points seen in previous frame
        int th;
        if(mSensor!=System::STEREO)
            th=15;
        else
            th=7;
        // --step4：根据上一帧特征点对应的3D点投影的位置缩小特征点匹配范围
        int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);
        //int lmatches = lmatcher.SearchByProjection(mCurrentFrame, mLastFrame);
    int lmatches2=0;
    if(mCurrentFrame.dealWithLine)
    {
        fill(mCurrentFrame.mvpMapLines.begin(),mCurrentFrame.mvpMapLines.end(), static_cast<MapLine*>(NULL));
        vector<MapLine*> vpMapLineMatches;
        lmatches2 = lmatcher.SearchByProjection(mpReferenceKF,mCurrentFrame,vpMapLineMatches);
        mCurrentFrame.mvpMapLines=vpMapLineMatches;
    }
        //vector<MapPoint*> vpMapPointMatches;

        //  int nmatches1 = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

        //lmatcher.SearchByProjection(mCurrentFrame, mLastFrame.mvpMapLines, th);
        //    cout << "tracking points = " << nmatches << endl;
        //    cout << "tracking lmatches = " << lmatches << endl;

        //double lmatch_ratio = lmatches*1.0/mCurrentFrame.mvKeylinesUn.size();
        //    cout << "lmatch_ratio = " << lmatch_ratio << endl;

        // If few matches, uses a wider window search
        // 如果跟踪的点少，则扩大搜索半径再来一次
        if(nmatches<20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
        }

        if(mCurrentFrame.dealWithLine)
        {
            if(nmatches<20 &&lmatches2<6) return false;
        }
        else
        {
            if(nmatches<20 )
                return false;
        }

    for(size_t i=0;i<mCurrentFrame.mvpMapLines.size();i++)
    {
        MapLine * ml=mCurrentFrame.mvpMapLines[i];
        if(ml)
        {
            //Eigen::Vector3d line_obs=mCurrentFrame.mvKeyLineFunctions[i];
            //cout<<"TRACKING i,line_obes"<<i<<","<<line_obs[0]<<","<<line_obs[1]<<","<<line_obs[2]<<endl;
            //cout<<"tracking:"<<ml->mWorldPos[0]<<","<<ml->mWorldPos[1]<<","<<ml->mWorldPos[2]<<endl;
        }
    }
        // Optimize frame pose with all matches
        // --step5: 优化位姿
        cout<<"tracking motion model,pose before opti"<<mCurrentFrame.mTcw<<endl;
        if(mCurrentFrame.dealWithLine)
            Optimizer::PoseOptimization(&mCurrentFrame, true);
        else
            Optimizer::PoseOptimization(&mCurrentFrame);
        cout<<"tracking motion model,pose after opti"<<mCurrentFrame.mTcw<<endl;
        // Discard outliers
        // --step6：优化位姿后剔除outlier的mvpMapPoints
        int nmatchesMap = 0;
        int nmatchesLineMap=0;

        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }

        for(int i=0; i<mCurrentFrame.NL;i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                if(mCurrentFrame.mvbLineOutlier[i])
                {
                    MapLine* pML=mCurrentFrame.mvpMapLines[i];
                    mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
                    mCurrentFrame.mvbLineOutlier[i]=false;
                    pML->mbTrackInView=false;
                    pML->mnLastFrameSeen=mCurrentFrame.mnId;
                    lmatches2--;
                }
                else if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                    nmatchesLineMap++;
            }

        }



        if(mbOnlyTracking)
        {
            mbVO = nmatchesMap<10;
            return nmatches>20;
        }

        if(mCurrentFrame.dealWithLine)
        {
            return (nmatchesMap >= 10 || nmatchesLineMap >= 6);
        }
        else
        {
            return nmatchesMap>=10;
        }

}

/**
 * @brief 对Local Map的MapPoints进行跟踪
 * 1.更新局部地图，包括局部关键帧和关键点
 * 2.对局部MapPoints进行投影匹配
 * 3.根据匹配对估计当前帧的姿态
 * 4.根据姿态剔除误匹配
 * @return true if success
 */
    bool Tracking::TrackLocalMap()
    {
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.

        UpdateLocalMap();

        SearchLocalPoints();

        // Optimize Pose
        Optimizer::PoseOptimization(&mCurrentFrame);
        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if(!mbOnlyTracking)
                    {
                        if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                            mnMatchesInliers++;
                    }
                    else
                        mnMatchesInliers++;
                }
                else if(mSensor==System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
            return false;

        if(mnMatchesInliers<10)
            return false;
        else
            return true;
    }

/**
 * @brief 类比TrackLocalMap函数，对局部地图的地图点和地图线进行跟踪
 * @return
 */
bool Tracking::TrackLocalMapWithLines()
{
        //cout<<"Tracking: track localmap with lines and points"<<endl;
        // step1：更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints，局部地图线mvpLocalMapLines
        UpdateLocalMap();
        //cout<<"Tracking: finish updatelocalmap"<<endl;

//    // step2：在局部地图中查找与当前帧匹配的MapPoints
//    SearchLocalPoints();
//    // step3: 在局部地图中查找与当前帧匹配的MapLines
//    SearchLocalLines();

        thread threadPoints(&Tracking::SearchLocalPoints, this);
        thread threadLines(&Tracking::SearchLocalLines, this);
        threadPoints.join();
        threadLines.join();

        //cout<<"Tracking: start Poseoptimization"<<endl;
        // step4：更新局部所有MapPoints和MapLines后对位姿再次优化
        cout<<"tracking localmap,pose before opti"<<endl<<mCurrentFrame.mTcw<<endl;
        Optimizer::PoseOptimization(&mCurrentFrame);
        cout<<"tracking localmap,pose after opti"<<mCurrentFrame.mTcw<<endl;
        mnMatchesInliers = 0;
        mnLineMatchesInliers = 0;

        // Update MapPoints Statistics
        // step5：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if(!mbOnlyTracking)
                    {
                        if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                            mnMatchesInliers++;
                    }
                    else
                        mnMatchesInliers++;
                }
                else if(mSensor==System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

            }
        }
        ///cout<<"Tracking: Maplines观测程度"<<endl;

        // 更新MapLines Statistics
        // step6：更新当前帧的MapLines被观测程度，并统计跟踪局部地图的效果
        for(int i=0; i<mCurrentFrame.NL; i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                if(!mCurrentFrame.mvbLineOutlier[i])
                {
                    mCurrentFrame.mvpMapLines[i]->IncreaseFound();
                    if(!mbOnlyTracking)
                    {
                        if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                            mnLineMatchesInliers++;
                    }
                    else
                        mnLineMatchesInliers++;
                }
                else if(mSensor==System::STEREO)
                    mCurrentFrame.mvpMapLines[i] = static_cast<MapLine*>(NULL);
            }
        }
        //cout<<"Tracking: finish Maplines观测程度"<<endl;
        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        // step7：决定是否跟踪成功
        if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
            return false;



        if(mnMatchesInliers<30)
            return false;
        else
            return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
    // "total matches = matches to map + visual odometry matches"
    // Visual odometry matches will become MapPoints if we insert a keyframe.
    // This ratio measures how many MapPoints we could create if we insert a keyframe.
    int nMap = 0; //nTrackedClose
    int nTotal= 0;
    int nNonTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                nTotal++;
                if(mCurrentFrame.mvpMapPoints[i]&&!mCurrentFrame.mvbOutlier[i])
                        nMap++;
                else
                    nNonTrackedClose++;
            }
        }
    }
    else
    {
        // There are no visual odometry matches in the monocular case
        nMap=1;
        nTotal=1;
    }

    const float ratioMap = (float)nMap/fmax(1.0f,nTotal);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    float thMapRatio = 0.35f;
    if(mnMatchesInliers>300)
        thMapRatio = 0.20f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || ratioMap<0.3f) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| ratioMap<thMapRatio) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;
    //所有的关键帧 都要提取线
    cout<<"dealwithline"<<mCurrentFrame.dealWithLine<<endl;
    if(!mCurrentFrame.dealWithLine)
    {
        cout<<"Tracking, non-blur  image NL"<<mCurrentFrame.NL<<endl;
        if(mCurrentFrame.NL==0)
        {
            cout<<"Tracking, extract lines for the current frame"<<endl;
            LineSegment* mpLineSegment;
            mpLineSegment->ExtractLineSegment(this->mImGray, mCurrentFrame.mvKeylinesUn, mCurrentFrame.mLdesc, mCurrentFrame.mvKeyLineFunctions);
            mCurrentFrame.mvDepthLine = vector<float>(mCurrentFrame.mvKeylinesUn.size(),-1.0f);
            mCurrentFrame.mvLines3D=vector<Vector6d>(mCurrentFrame.mvKeylinesUn.size(), static_cast<Vector6d>(NULL));
            for(int i=0; i<mCurrentFrame.mvKeylinesUn.size(); ++i)	{ // each line
                double len = cv::norm(mCurrentFrame.mvKeylinesUn[i].getStartPoint() - mCurrentFrame.mvKeylinesUn[i].getEndPoint());
                vector<cv::Point3d> pts3d;
                double numSmp = (double) min((int)len, 100); //number of line points sampled

                //cout<<"numSmp:"<<numSmp<<endl;

                pts3d.reserve(numSmp);//预留数量

                for(int j=0; j<=numSmp; ++j) {
                    // use nearest neighbor to querry depth value
                    // assuming position (0,0) is the top-left corner of image, then the
                    // top-left pixel's center would be (0.5,0.5)
                    cv::Point2d pt = mCurrentFrame.mvKeylinesUn[i].getStartPoint() * (1-j/numSmp) + mCurrentFrame.mvKeylinesUn[i].getEndPoint()* (j/numSmp);

                    //cout<<"pt"<<pt.x<<","<<pt.y<<endl;
                    if(pt.x<0 || pt.y<0 || pt.x >= this->mImDepth.cols || pt.y >= this->mImDepth.rows ) continue;
                    int row, col; // nearest pixel for pt
                    if((floor(pt.x) == pt.x) && (floor(pt.y) == pt.y)) {// boundary issue
                        col = max(int(pt.x-1),0);
                        row = max(int(pt.y-1),0);
                    } else {
                        col = int(pt.x);
                        row = int(pt.y);
                    }
                    //cout<<"col:"<<col<<","<<"row:"<<row<<endl;
                    float d=-1;
                    if(this->mImGray.at<float>(row,col) <=0.01) { // no depth info
                        //cout<<"no depth info"<<endl;
                        continue;
                    }
                    else {
                        d = this->mImGray.ptr<float>(row)[col];
                    }
                    cv::Point3d p;

                    //cout<<"col:"<<col<<", row:"<<row<<endl;
                    //cout<<"cx:"<<cx<<",fx"<<fx<<endl;
                    // 计算这个点的空间坐标
                    p.z = d;
                    p.x = (col - mCurrentFrame.cx) * p.z *mCurrentFrame.invfx;//K.at<float>(0,0);//invfx;
                    p.y = (row - mCurrentFrame.cy) * p.z *mCurrentFrame.invfy;//K.at<float>(1,1);//invfy;
                    //cout<<"x:"<<p.x<<",y:"<<p.y<<",z:"<<p.z<<endl;

                    //mv3DLineforMap.push_back(p);
                    //origPoints_xyzrgb<<p.x<<" "<<p.y<<" "<<p.z<<endl;
                    pts3d.push_back(p);

                }
                //如果 点数量少于 10或者 ，就抛弃该线
                if (pts3d.size() < 10.0)//sysPara.ratio_of_collinear_pts
                    continue;

                RandomLine3d tmpLine;
#if 1
                vector<RandomPoint3d> rndpts3d;
                rndpts3d.reserve(pts3d.size());
                //cout<<"pts3d size:"<<pts3d.size()<<endl;
                // compute uncertainty of 3d points
                for(int j=0; j<pts3d.size();++j) {
                    //cout<<"ptsxyz"<<pts3d[j].x<<","<<pts3d[j].y<<","<<pts3d[j].z<<endl;
                    rndpts3d.push_back(compPt3dCov(pts3d[j], mCurrentFrame.mK, 1));
                    //cout<<"Du[0]:"<<compPt3dCov(pts3d[j], mK, 1).DU[0]<<endl;
                }
                // using ransac to extract a 3d line from 3d pts
                tmpLine = extract3dline_mahdist(rndpts3d);
#else
                //计算这个线的参数
        tmpLine = extract3dline(pts3d,origPoints,optiPoints);
#endif
                //cout<<"jieshu"<<endl;
                //修复这条直线上的点


                if(tmpLine.pts.size()/len > 0.4/*sysPara.ratio_of_collinear_pts*/	&&
                   cv::norm(tmpLine.A - tmpLine.B) >0.02) { // sysPara.line3d_length_thresh
                    //this line is reliable
                    mCurrentFrame.mvDepthLine[i]= 1.0f;
                    cout<<"gaibian line 3d"<<endl;
                    mCurrentFrame.mvLines3D[i]<<tmpLine.A.x,tmpLine.A.y,tmpLine.A.z,tmpLine.B.x,tmpLine.B.y,tmpLine.B.z;
                }

                /*if(1==mCurrentFrame.mvDepthLine[i])
                {
                    // rebuild this line in depth map

                    for(int j=0; j<=numSmp; ++j)
                    {
                        cv::Point2d pt = mCurrentFrame.mvKeylinesUn[i].getStartPoint() * (1-j/numSmp) + mCurrentFrame.mvKeylinesUn[i].getEndPoint() * (j/numSmp);
                        if(pt.x<0 || pt.y<0 || pt.x >= this->mImGray.cols || pt.y >= this->mImGray.rows ) continue;

                        int row, col; // nearest pixel for pt
                        if((floor(pt.x) == pt.x) && (floor(pt.y) == pt.y)) {// boundary issue
                            col = max(int(pt.x-1),0);
                            row = max(int(pt.y-1),0);
                        }
                        else {
                            col = int(pt.x);
                            row = int(pt.y);
                        }


                        float d=this->mImGray.ptr<float>(row)[col];
                        cv::Point3d p;
                        if(d<=0.01) continue;
                        else
                        {
                            p.z=d;
                            if (p.z < 0.01 || p.z>10){cout<<"Frame: invalid p.z"<<endl;continue;}

                            p.x = (col - mCurrentFrame.cx) * p.z *mCurrentFrame.invfx;
                            p.y = (row - mCurrentFrame.cy) * p.z *mCurrentFrame.invfy;
                            p.z=mCurrentFrame.fy*mCurrentFrame.fx*(tmpLine.director.y*tmpLine.mid.x-
                                       tmpLine.director.x*tmpLine.mid.y)/
                                (mCurrentFrame.fy*tmpLine.director.y*(col-mCurrentFrame.cx)-
                                        mCurrentFrame.fx*tmpLine.director.x*(row-mCurrentFrame.cy));
                            //cout<<p.x<<", "<<p.y<<", "<<p.z<<endl;
                            mCurrentFrame.mv3DLineforMap.push_back(p);
                        }
                        //tmpLine.pts.push_back(p);


                    }
                }*/
            }

            mCurrentFrame.NL=mCurrentFrame.mvKeylinesUn.size();
            mCurrentFrame.mvpMapLines=vector<MapLine *>(mCurrentFrame.NL,static_cast<MapLine*>(NULL));
            mCurrentFrame.mvbLineOutlier=vector<bool>(mCurrentFrame.NL,false);
            cout<<"Tracking, extract lines from non-blur KF,NL"<<mCurrentFrame.NL<<endl;

        }

    }

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        cout<<"Tracking: newKeyframe: RGBD"<<endl;

        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vector<pair<float, int>> vLineDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        for(int i=0;i<mCurrentFrame.NL;i++)
        {
            float isDepthGood= mCurrentFrame.mvDepthLine[i];
            //cout<<"Tracking: StereoInitialization:线是否存在:"<<isDepthGood<<endl;
            //如果存在， 执行程序
            cout<<"tracking,isDepthGood"<<isDepthGood<<endl;
            if(isDepthGood==1)
            {
                vLineDepthIdx.push_back(make_pair(isDepthGood,i));
            }
        }
        cout<<"Tracking finish adding vDepthID and vLineDepthId"<<endl;


        if(!vLineDepthIdx.empty())
        {
            //sort(vLineDepthIdx.begin(),vLineDepthIdx.end());

            int nLines = 0;
            for(size_t j=0; j<vLineDepthIdx.size();j++)
            {
                int i = vLineDepthIdx[j].second;

                bool bCreateNew = false;

                MapLine* pMP = mCurrentFrame.mvpMapLines[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapLines[i] = static_cast<MapLine*>(NULL);
                }

                if(bCreateNew)
                {
                    Vector6d line3D= mCurrentFrame.obtain3DLine(i);//mvLines3D[i];
                    MapLine *pNewML=new MapLine(line3D,pKF,mpMap);
                    pNewML->AddObservation(pKF,i);
                    pKF->AddMapLine(pNewML,i);
                    pNewML->ComputeDistinctiveDescriptors();
                    pNewML->UpdateAverageDir();
                    mpMap->AddMapLine(pNewML);
                    mCurrentFrame.mvpMapLines[i]=pNewML;
                    nLines++;
                }
                else
                {
                    nLines++;
                }

                if(nLines>20)
                    break;
            }
        }
        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);
   
    // TO DO
    //mpPointCloudMapping->insertKeyFrame( pKF, this->mImRGB, this->mImDepth );

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++; //将要match的
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::SearchLocalLines()
{
        // step1：遍历在当前帧的mvpMapLines，标记这些MapLines不参与之后的搜索，因为当前的mvpMapLines一定在当前帧的视野中
        for(vector<MapLine*>::iterator vit=mCurrentFrame.mvpMapLines.begin(), vend=mCurrentFrame.mvpMapLines.end(); vit!=vend; vit++)
        {
            MapLine* pML = *vit;
            if(pML)
            {
                if(pML->isBad())
                {
                    *vit = static_cast<MapLine*>(NULL);
                } else{
                    // 更新能观测到该线段的帧数加1
                    pML->IncreaseVisible();
                    // 标记该点被当前帧观测到
                    pML->mnLastFrameSeen = mCurrentFrame.mnId;
                    // 标记该线段将来不被投影，因为已经匹配过
                    pML->mbTrackInView = false;
                }
            }
        }

        int nToMatch = 0;

        // step2：将所有局部MapLines投影到当前帧，判断是否在视野范围内，然后进行投影匹配
        for(vector<MapLine*>::iterator vit=mvpLocalMapLines.begin(), vend=mvpLocalMapLines.end(); vit!=vend; vit++)
        {
            MapLine* pML = *vit;

            // 已经被当前帧观测到MapLine，不再判断是否能被当前帧观测到
            if(pML->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if(pML->isBad())
                continue;

            // step2.1：判断LocalMapLine是否在视野内
            if(mCurrentFrame.isInFrustum(pML, 0.6))
            {
                // 观察到该点的帧数加1，该MapLine在某些帧的视野范围内
                pML->IncreaseVisible();
                nToMatch++;
            }
        }

        if(nToMatch>0)
        {
            LSDmatcher matcher;
            int th = 1;

            if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
                th=5;

            matcher.SearchByProjection(mCurrentFrame, mvpLocalMapLines, th);
        }
    }


void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    mpMap->SetReferenceMapLines(mvpLocalMapLines);

    // Update
    UpdateLocalKeyFrames();
    cout<<"the size of local keyframe"<<mvpLocalKeyFrames.size()<<endl;

    UpdateLocalPoints();
    UpdateLocalLines();
}

void Tracking::UpdateLocalLines()
{
        cout<<"Tracking: UpdateLocalLines()"<<endl;
        // step1：清空局部MapLines
        mvpLocalMapLines.clear();

        // step2：遍历局部关键帧mvpLocalKeyFrames
        for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
        {
            KeyFrame* pKF = *itKF;
            const vector<MapLine*> vpMLs = pKF->GetMapLineMatches();

            //step3：将局部关键帧的MapLines添加到mvpLocalMapLines
            for(vector<MapLine*>::const_iterator itML=vpMLs.begin(), itEndML=vpMLs.end(); itML!=itEndML; itML++)
            {
                MapLine* pML = *itML;
                if(!pML)
                    continue;
                if(pML->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                    continue;
                if(!pML->isBad())
                {
                    mvpLocalMapLines.push_back(pML);
                    pML->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
            }
        }
        cout<<"Tracking: Finish updateLocallines"<<endl;
    }

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    cout<<"Tracking:localization"<<endl;
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    cout<<"Tracking,vpCandidateKFs"<<vpCandidateKFs.size()<<endl;
    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

bool Tracking::LocalizationforPL()
{
        cout<<"Tracking:localization"<<endl;
        // Compute Bag of Words Vector
        mCurrentFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

        cout<<"Tracking,vpCandidateKFs"<<vpCandidateKFs.size()<<endl;
        if(vpCandidateKFs.empty())
            return false;

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75,true);
        LSDmatcher lmatcher;

        vector<PnPsolver*> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint*> > vvpMapPointMatches;
        vector<vector<MapLine*>>   vvpMapLineMatches;
        vvpMapPointMatches.resize(nKFs);
        vvpMapLineMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates=0;

        for(int i=0; i<nKFs; i++)
        {
            KeyFrame* pKF = vpCandidateKFs[i];
            if(pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
                int lmatches = lmatcher.SearchByProjection(pKF,mCurrentFrame,vvpMapLineMatches[i]);
                if(nmatches<15 && lmatches<6)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }


        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9,true);
        //LSDmatcher lmatcher;
        while(nCandidates>0 && !bMatch)
        {
            for(int i=0; i<nKFs; i++)
            {
                if(vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver* pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if(bNoMore)
                {
                    vbDiscarded[i]=true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if(!Tcw.empty())
                {
                    Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint*> sFound;

                    const int np = vbInliers.size();

                    for(int j=0; j<np; j++)
                    {
                        if(vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j]=NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if(nGood<10)
                        continue;

                    for(int io =0; io<mCurrentFrame.N; io++)
                        if(mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    if(nGood<50)
                    {
                        int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                        if(nadditional+nGood>=50)
                        {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if(nGood>30 && nGood<50)
                            {
                                sFound.clear();
                                for(int ip =0; ip<mCurrentFrame.N; ip++)
                                    if(mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                                // Final optimization
                                if(nGood+nadditional>=50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for(int io =0; io<mCurrentFrame.N; io++)
                                        if(mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io]=NULL;
                                }
                            }
                        }
                    }


                    // If the pose is supported by enough inliers stop ransacs and continue
                    if(nGood>=50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if(!bMatch)
        {
            return false;
        }
        else
        {
            mnLastRelocFrameId = mCurrentFrame.mnId;
            return true;
        }

    }

void Tracking::Reset()
{
    mpViewer->RequestStop();

    cout << "System Reseting" << endl;
    while(!mpViewer->isStopped())
        usleep(3000);

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    //mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM

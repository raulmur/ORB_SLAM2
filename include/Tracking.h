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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

/** \brief Tracker. It receives a frame and computes the associated camera pose.
*
* Tracker is one of the main threads of System (it runs within the main thread).
* It also decides when to insert a new keyframe, create some new MapPoints and
* performs relocalization if tracking fails. Relocalization is important if tracking
* is lost due to sudden large motions or reinitialization of the system. The
* Tracking thread produces a KeyFrame when it is required.
*
* Whenever a frame is grabbed from the camera, the main stages in tracking are:
* * Pre-process input to get ORB features and triangulate points, where possible (implemented in Frame)
* * Camera position estimation or relocation (Tracking::Track)
**/
class Tracking
{

public:
    /// Constructor
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    /// Preprocess the stereo input and call Track(). Extract features and performs stereo matching. Perform tracking.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);

    /// Preprocess the RGB-D input and call Track(). Extracts features. Perform tracking.
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);

    /** \brief Preprocess the monocular input and call Track(). Extracts features, does feature point matching. Perform tracking.
    * 
    * Monocular cameras are simple but cannot produce the initial map from the first frame. Techniques
    * are needed to triangule features based on multiple frames. Monocular cameras can also suffer from 
    * scale drift. **/
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    /** \brief Load new settings
    *
    * The focal lenght should be similar or scale prediction will fail when projecting points
    * TODO: Modify MapPoint::PredictScale to take into account focal length **/
    void ChangeCalibration(const string &strSettingPath);

    /// Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    /// Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    /// Current tracking state
    eTrackingState mState;
    eTrackingState mLastProcessedState;

    /// Input sensor type
    int mSensor;

    /// Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    /// True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    /** \brief Main tracking function. It is independent of the input sensor.
    *
    * This is called whenever a new frame is grabbed. The tracking procedure is:
    * * Update camera position based on motion model, if available (Tracking::TrackWithMotionModel)
    * * Refine camera position based on all matches between mCurrentFrame and mLastFrame, identify outliers (Tracking::TrackWithMotionModel)
    * * Refine camera position camera position relative to reference frame by BoW matched points (Tracking::TrackReferenceKeyFrame)
    * * Refine the camera position based on local mapping data (Tracking::TrackLocalMap)
    * * Check if we need to add a new KeyFrame (Tracking::NeedNewKeyFrame)
    * If tracking fails, fall back to relocalization.
    **/
    void Track();

    /// Map initialization for stereo and RGB-D
    void StereoInitialization();

    /// Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    ///Check if the last frame's MapPoints have been modified by LocalMapping
    void CheckReplacedInLastFrame();

    ///Check current frame is similar to reference frame, estimate camera position from BoW matched points
    bool TrackReferenceKeyFrame();

    /// \brief Update last frame pose according to its reference keyframe, and select "visual odometery" points
    ///
    /// Visual odometery points are all track points with distance < mThDepth. If there are less than 100 points,
    /// select the nearest 100.
    void UpdateLastFrame();

	/// \brief Initial pose estimation from motion and previous frame
    ///
    /// Apply motion model, find feature matches from last frame, estimate new camera position, idenfity outliers.
    bool TrackWithMotionModel();

    /// Relocalization based on bag of words is performed when tracking is lost
    bool Relocalization();

    /// \brief Determine which KeyFrames and MapPoints are currently local to our camera position
    ///
    /// Used by Tracking::TrackLocalMap
    void UpdateLocalMap();

    /// \brief Determine which MapPoints are local, based on the current local key frames
    ///
    /// This updates Tracking::mvpLocalMapPoints
    void UpdateLocalPoints();

    /// \brief Determine which KeyFrames are currently local to our camera position.
    ///
    /// This updates Tracking::mvpLocalKeyFrames
    void UpdateLocalKeyFrames();

    /** \brief Estimate new camera position based on local map by minimizing reprojection error.
    *
    * We have an estimation of the camera pose and some map points tracked in the frame.
    * We retrieve the local map and try to find matches to points in the local map.
    *
    * The procedure is:
    * * Find local key frames and map points. (Tracking::UpdateLocalMap)
    * * Project local points to camera space, discard any that are not expected to be in view of the camera, perform matching. (Tracking::SearchLocalPoints)
    * * Estimate camera pose (Optimizer::PoseOptimization)
    * * Update MapPoint statistics.
    * * Check if tracking is OK
    */
    bool TrackLocalMap();

    /** \brief Determine local points which should be currently visible to the camera and perform matching.
    *
    * Used by Tracking::TrackLocalMap. The procedure is:
    * * Skip points already matched
    * * Compute angle between mean viewing ray and camera direction. Discard points with oblique view. (Frame::isInFrustum)
    * * Compute distance to points. Discard those outside the scale invariance range of ORB descriptor. (Frame::isInFrustum)
    * * Compute expected scale of map point (Frame::isInFrustum)
    * * Match points based on ORB descriptor
    **/
    void SearchLocalPoints();

    /** \brief Check if a new key frame should be inserted
    *
    * The system uses the approach of frequently adding key frames, which are later culled
    * in LocalMapping::KeyFrameCulling. A key frame is added if the number of close points
    * drops below 100 and at least 70 new close points could be added. If required,
    * Tracking::CreateNewKeyFrame is called.
    */
    bool NeedNewKeyFrame();

    /// Create a new KeyFrame based on Tracking::mCurrentFrame and insert with LocalMapping::InsertKeyFrame
    void CreateNewKeyFrame();

    /// In case of performing only localization, this flag is true when there are no matches to
    /// points in the map. Still tracking will continue if there are enough matches with temporal points.
    /// In that case we are doing visual odometry. The system will try to do relocalization to recover
    /// "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB extractors
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    ///BoW voculabulary
    ORBVocabulary* mpORBVocabulary;

    ///Database of KeyFrames
    KeyFrameDatabase* mpKeyFrameDB;

    /// Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF; ///< Current local map reference KeyFrame
    std::vector<KeyFrame*> mvpLocalKeyFrames; ///< Current local map KeyFrames, found by Tracking::UpdateLocalKeyFrames
    std::vector<MapPoint*> mvpLocalMapPoints; ///< Current local map MapPoint, found by Tracking::UpdateLocalPoints
    
    /// System
    System* mpSystem;
    
    //Drawers for visualization
    Viewer* mpViewer; ///< Visualization GUI
    FrameDrawer* mpFrameDrawer; ///< Visualization of current frame
    MapDrawer* mpMapDrawer; ///< Visualization of current map

    ///Current map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    /// \brief Threshold to separate close/far points
    ///
    /// Points seen as close by the stereo/RGBD sensor are considered reliable
    /// and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    /// For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    /// Current matches in frame
    int mnMatchesInliers;

    // Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    /// Motion Model
    cv::Mat mVelocity;

    /// Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H

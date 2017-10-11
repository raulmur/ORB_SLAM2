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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;

/** \brief Loop Closer. It searches for loops with every new keyframe. 
*
* LoopClosing is one of the main threads of System.
* If there is a loop it performs a pose graph optimization and 
* full bundle adjustment (in a new thread) afterwards. Loop closure depends
* on recognizing places that have been previously visited. This
* reduces accumulated drift in the mapping model.
*
* The main stages in LoopClosing are:
* * loop detection (LoopClosing::DetectLoop)
* * loop correction (LoopClosing::CorrectLoop)
*
* RGB-D and Stereo sensors do not suffer from scale drift, so the observations
* can be treated as a rigid body in the pose graph optimization (LoopClosing::mbFixScale 
* is true). The global optimization
* can take some time, so it operates in a separate thread while the system continues
* to operate. This brings the challenge of merging the optimized model with current
* state of the map. New KeyFrames are corrected based on the correction applied to their
* associated original KeyFrame.
*/
class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

public:
    /// \brief Constructor
    /// \param bFixScale if is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono).
    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    /// Main function
    void Run();

    /// \brief Receive new frame from LocalMapper for loop closure checking
    ///
    /// Called from LocalMapping::Run
    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();

    /// This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    /// Check if frames are waiting in LoopClosing::mlpLoopKeyFrameQueue
    bool CheckNewKeyFrames();

	/// Find similar Keyframes using bag of words.
    /// 
    /// Looks up similar KeyFrames using KeyFrameDatabase::DetectLoopCandidates(). We 
    /// must detect a consistent loop in several consecutive keyframes to accept it.
    bool DetectLoop();

    /// \brief Compute similarity transformation
    ///
    /// Finds the relative position of two coordinate systems based on a set of observations. 
    /// Optimizer is implemented in Sim3Solver.
    bool ComputeSim3();

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    /** \brief Perform a loop closure.
    *
	* The main stages in loop correction are:
	* * Loop fusion (mainly LoopClosing::SearchAndFuse)
	* * Optimize essential graph (Optimizer::OptimizeEssentialGraph)
	* * Launch a separate thread to do a full bundle ajustment (LoopClosing::RunGlobalBundleAdjustment)
	* * Merge the corrections into the current map state (LoopClosing::RunGlobalBundleAdjustment)
    */
    void CorrectLoop();

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    /// Loop detector parameters
    float mnCovisibilityConsistencyTh;

    /// Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    /// Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    /// Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    bool mnFullBAIdx;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H

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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;

/** \brief Bundle adjustment and optimization functions.
*
* Bundle adjustment is used for
* * Tracking (motion only bundle adjustment) Optimizer::PoseOptimization
* * LocalMapping (optimizing the local window of each KeyFrame and each local MapPoint) Optimizer::LocalBundleAdjustment
* * Global bundle adjustment after loop closure. GlobalBundleAdjustemnt::LocalBundleAdjustment
*
* Levenberg-Marquadt optimization is used for bundle adjustment (from the g2o library).
*/
class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);

    ///\brief Global bundle adjustment after loop closure has been detected.
    ///
    /// The origin KeyFrame remains fixed. All other KeyFrames are optimized by bundle adjustment.
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);

    //\brief Optimizing the local window of covisible KeyFrame and all MapPoint objects in those KeyFrame.
    //
    // This is called when new KeyFrame is added. All other KeyFrames contribute to the cost
    // function but remain constant.
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);

    ///Tracking (motion only bundle adjustment) by minimizing reprojection error
    int static PoseOptimization(Frame* pFrame);

    /// \brief Used by LoopClosing to optimize essential graph (relative camera positions) of KeyFrame network.
    /// \param bFixScale if is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono).
    ///
    /// Called from LoopClosing::CorrectLoop. This distributes any drift error over the
    /// entire network. Only the essential graph is optimized, rather than every edge in
    /// the network.
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    /// \brief Used by LoopClosing.
    /// \param bFixScale If true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)    
    /// 
    /// Called from LoopClosing::ComputeSim3.
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H

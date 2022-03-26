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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <mutex>

namespace ORB_SLAM2
{

    long unsigned int KeyFrame::nNextId = 0;

    KeyFrame::KeyFrame(
        Frame &F,
        Map *pMap,
        KeyFrameDatabase *pKFDB)
        // 这里做了非常多的赋值操作，KeyFrame和Frame的基本是对应的
        // Frame里面由的KeyFrame里面也有
        : mnFrameId(F.mnId),
          mTimeStamp(F.mTimeStamp),
          mnGridCols(FRAME_GRID_COLS),
          mnGridRows(FRAME_GRID_ROWS),
          mfGridElementWidthInv(F.mfGridElementWidthInv),
          mfGridElementHeightInv(F.mfGridElementHeightInv),
          mnTrackReferenceForFrame(0),
          mnFuseTargetForKF(0),
          mnBALocalForKF(0),
          mnBAFixedForKF(0),
          mnLoopQuery(0),
          mnLoopWords(0),
          mnRelocQuery(0),
          mnRelocWords(0),
          mnBAGlobalForKF(0),
          fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy),
          invfx(F.invfx), invfy(F.invfy),
          mbf(F.mbf), mb(F.mb),
          mThDepth(F.mThDepth),
          N(F.N),
          mvKeys(F.mvKeys),
          mvKeysUn(F.mvKeysUn),
          mvuRight(F.mvuRight), mvDepth(F.mvDepth),
          mDescriptors(F.mDescriptors.clone()),
          mBowVec(F.mBowVec), mFeatVec(F.mFeatVec),
          mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
          mfLogScaleFactor(F.mfLogScaleFactor),
          mvScaleFactors(F.mvScaleFactors),
          mvLevelSigma2(F.mvLevelSigma2),
          mvInvLevelSigma2(F.mvInvLevelSigma2),
          mnMinX(F.mnMinX), mnMinY(F.mnMinY),
          mnMaxX(F.mnMaxX), mnMaxY(F.mnMaxY),
          mK(F.mK),
          mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
          mpORBvocabulary(F.mpORBvocabulary),
          mbFirstConnection(true), mpParent(NULL),
          mbNotErase(false), mbToBeErased(false), mbBad(false),
          mHalfBaseline(F.mb / 2), mpMap(pMap)
    {
        mnId = nNextId++;

        mGrid.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++)
        {
            mGrid[i].resize(mnGridRows);
            for (int j = 0; j < mnGridRows; j++)
                mGrid[i][j] = F.mGrid[i][j];
        }

        SetPose(F.mTcw);
    }

    void KeyFrame::ComputeBoW()
    {
        // 用于计算词袋模型
        if (mBowVec.empty() || mFeatVec.empty())
        {
            // 这里的mDescription是在KeyFrame的构造函数里进行初始化的，直接把Frame的mDescriptors复制过来了
            // 这一行其实只是进行了数据转换，更具体地说是矩阵的按行拆分
            // 因为在Frame里，描述子是用一个大的Mat表示的，每一行表示一个特征点，但在KeyFrame中按行将这个大矩阵进行了拆分
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            // Feature vector associate features with nodes in the 4th level (from leaves up)
            // We assume the vocabulary tree has 6 levels, change the 4 otherwise
            // 这里就是调用了BoW的相关API，将当前vector表示的特征点描述子转成BoW向量赋给KeyFrame成员变量
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    void KeyFrame::SetPose(const cv::Mat &Tcw_)
    {
        unique_lock<mutex> lock(mMutexPose);
        // 将新的位姿拷贝到成员变成Tcw
        Tcw_.copyTo(Tcw);
        // 基于新的变换矩阵计算R、t
        cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
        cv::Mat Rwc = Rcw.t();  // 旋转矩阵是正交矩阵（正交矩阵的转置等于它的逆）
        Ow = -Rwc * tcw;    // 计算变换过程对应的平移部分，其实对应的也就是相机中心在世界坐标系下的坐标

        // 计算world到camera的变换矩阵
        Twc = cv::Mat::eye(4, 4, Tcw.type());
        // 先将上面转置得到的代表逆旋转的旋转矩阵对应赋值赋过来
        Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
        // 将上面计算好的平移部分直接赋值
        Ow.copyTo(Twc.rowRange(0, 3).col(3));
        // 计算center
        cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
        // 计算Cw ?
        Cw = Twc * center;
    }

    // 返回camera到world的4*4的变换矩阵
    cv::Mat KeyFrame::GetPose()
    {
        unique_lock<mutex> lock(mMutexPose);
        // 返回的是一个Mat类型的矩阵，表示关键帧当前的位姿
        // 这个变换矩阵其实是一个4*4，由R、t构成的特殊欧式群SE3
        return Tcw.clone();
    }

    // 返回world到camera的4*4的变换矩阵（与GetPose正好相反）
    cv::Mat KeyFrame::GetPoseInverse()
    {
        // 线程独占锁
        unique_lock<mutex> lock(mMutexPose);
        // 返回从world到camera的变换矩阵Twc，和GetPose函数正好相反
        return Twc.clone();
    }

    // 返回相机中心坐标(4 * 1矩阵 ? )
    cv::Mat KeyFrame::GetCameraCenter()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Ow.clone();
    }

    cv::Mat KeyFrame::GetStereoCenter()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Cw.clone();
    }

    // 获取camera到world的旋转矩阵（变换矩阵的前三行前三列）；Tcw：camera到world的旋转矩阵
    cv::Mat KeyFrame::GetRotation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0, 3).colRange(0, 3).clone();
    }

    cv::Mat KeyFrame::GetTranslation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0, 3).col(3).clone();
    }

    void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
    {
        {
            unique_lock<mutex> lock(mMutexConnections);
            // mConnectedKeyFrameWeights是KeyFrame的map类型的成员变量，存放的是关键帧以及对应的观测次数
            // 这里利用map类的count函数进行了统计，统计某个关键帧对应的观测次数
            if (!mConnectedKeyFrameWeights.count(pKF))
            // 如果当前map中还没有pKF的权重，就把传入的weight赋给它
                mConnectedKeyFrameWeights[pKF] = weight;
            else if (mConnectedKeyFrameWeights[pKF] != weight)
            // 如果说当前map中已经有了权重值且不等于传入的weight，就把传入的权重值赋过来
                mConnectedKeyFrameWeights[pKF] = weight;
            else
                return;
        }

        UpdateBestCovisibles();
    }

    void KeyFrame::UpdateBestCovisibles()
    {
        unique_lock<mutex> lock(mMutexConnections);
        vector<pair<int, KeyFrame *>> vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
            vPairs.push_back(make_pair(mit->second, mit->first));

        sort(vPairs.begin(), vPairs.end());
        list<KeyFrame *> lKFs;
        list<int> lWs;
        for (size_t i = 0, iend = vPairs.size(); i < iend; i++)
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
    }

    set<KeyFrame *> KeyFrame::GetConnectedKeyFrames()
    {
        unique_lock<mutex> lock(mMutexConnections);
        set<KeyFrame *> s;
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end(); mit++)
            s.insert(mit->first);
        return s;
    }

    vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames()
    {
        unique_lock<mutex> lock(mMutexConnections);
        return mvpOrderedConnectedKeyFrames;
    }

    vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
    {
        unique_lock<mutex> lock(mMutexConnections);
        if ((int)mvpOrderedConnectedKeyFrames.size() < N)
            return mvpOrderedConnectedKeyFrames;
        else
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);
    }

    vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w)
    {
        unique_lock<mutex> lock(mMutexConnections);

        if (mvpOrderedConnectedKeyFrames.empty())
            return vector<KeyFrame *>();

        vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w, KeyFrame::weightComp);
        if (it == mvOrderedWeights.end())
            return vector<KeyFrame *>();
        else
        {
            int n = it - mvOrderedWeights.begin();
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
        }
    }

    int KeyFrame::GetWeight(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (mConnectedKeyFrameWeights.count(pKF))
            return mConnectedKeyFrameWeights[pKF];
        else
            return 0;
    }

    void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
    {
        // 首先加一个线程同步锁防止冲突
        // 然后就是将我们传入的MapPoint指针赋给KeyFrame的成员变量mvMapPoints的对应位置
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = pMP;
    }

    void KeyFrame::EraseMapPointMatch(const size_t &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }

    void KeyFrame::EraseMapPointMatch(MapPoint *pMP)
    {
        int idx = pMP->GetIndexInKeyFrame(this);
        if (idx >= 0)
            mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }

    void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP)
    {
        mvpMapPoints[idx] = pMP;
    }

    set<MapPoint *> KeyFrame::GetMapPoints()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        set<MapPoint *> s;
        for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++)
        {
            if (!mvpMapPoints[i])
                continue;
            MapPoint *pMP = mvpMapPoints[i];
            if (!pMP->isBad())
                s.insert(pMP);
        }
        return s;
    }

    int KeyFrame::TrackedMapPoints(const int &minObs)
    {
        unique_lock<mutex> lock(mMutexFeatures);

        int nPoints = 0;
        // 最小观测次数大于0?如果是的话为true，否则为false，这里传入的为1，所以为true
        const bool bCheckObs = minObs > 0;
        // 对于关键帧中的所有特征点
        for (int i = 0; i < N; i++)
        {
            // 根据索引获取对应 的地图点
            MapPoint *pMP = mvpMapPoints[i];
            // 如果对应地图点存在
            if (pMP)
            {
                // 且该地图点不是坏的
                if (!pMP->isBad())
                {
                    // 是否检查地图点观测数量，如果传入的参数minObs大于0的话就检查
                    if (bCheckObs)
                    {
                        // 这一行的意思是假如关键帧上的某个特征点对应的地图点存在且不是坏的
                        // 进一步判断这个地图点的观测数量是否大于给定的阈值
                        // 如果大于阈值，认为是ok的，数量加1，否则不加
                        if (mvpMapPoints[i]->Observations() >= minObs)
                            nPoints++;
                    }
                    else
                        nPoints++;
                }
            }
        }
        // 最后，返回符合条件的地图点数量
        return nPoints;
    }

    vector<MapPoint *> KeyFrame::GetMapPointMatches()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        // 返回关键帧中对应的地图点vector
        return mvpMapPoints;
    }

    MapPoint *KeyFrame::GetMapPoint(const size_t &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        // 返回对应的地图点
        return mvpMapPoints[idx];
    }

    void KeyFrame::UpdateConnections()
    {
        map<KeyFrame *, int> KFcounter;

        vector<MapPoint *> vpMP;

        {
            unique_lock<mutex> lockMPs(mMutexFeatures);
            // 将vector成员变量mvpMapPoints赋给临时变量，这个vector里存放的是当前关键帧对应的地图点
            // 在Tracking的CreatInitialMapMonocular函数里调用了KeyFrame的成员函数AddMapPoint将地图点添加到了mvpMapPoints中
            vpMP = mvpMapPoints;
        }

        // For all map points in keyframe check in which other keyframes are they seen
        // Increase counter for those keyframes
        // 简单来说就是对于当前帧中的地图点，依次遍历看看某个点是否在其它关键帧中也被看到了
        // 这样可以统计每个地图点除了当前关键帧，被检测到的次数
        for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++)
        {
            // 将迭代器变量指针赋给pMP
            MapPoint *pMP = *vit;

            // 如果为空跳过本次循环
            if (!pMP)
                continue;

            // 如果地图点是坏的，跳过本次循环
            // 至于说什么是坏的，在isBad函数中调用了两个线程锁，并没有完全理解
            if (pMP->isBad())
                continue;

            // 获取这个地图点的观测，也就是之前关联的关键帧
            map<KeyFrame *, size_t> observations = pMP->GetObservations();

            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                // 如果关键帧ID和本次关键帧相同，就不累加了，跳过，对于其它的累加
                if (mit->first->mnId == mnId)
                    continue;
                // 这里需要注意累加的方式
                // KFcounter是一个map，mit也是一个map类型的迭代变量，mit->first获取的是对应的KeyFrame指针
                // 这里将KeyFrame指针作为索引，或者说“键”，去查找对应的int类型的数（值），找到后对其进行累加
                KFcounter[mit->first]++;
            }
        }

        // This should not happen
        if (KFcounter.empty())
            return;

        // If the counter is greater than threshold add connection
        // In case no keyframe counter is over threshold add the one with maximum counter
        int nmax = 0;
        KeyFrame *pKFmax = NULL;
        int th = 15;    // 观测次数阈值设置为15，如果观测次数大于15则增加连接

        vector<pair<int, KeyFrame *>> vPairs;
        vPairs.reserve(KFcounter.size());
        for (map<KeyFrame *, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++)
        {
            // 这个if语句其实是在寻找观测次数最多的那个关键帧
            if (mit->second > nmax)
            {
                nmax = mit->second;
                pKFmax = mit->first;
            }
            // 这个if语句是在判断某个关键帧的观测次数是否大于阈值，是的话就添加
            if (mit->second >= th)
            {
                vPairs.push_back(make_pair(mit->second, mit->first));
                // 添加连接，传入两个参数，KeyFrame指针和观测次数
                (mit->first)->AddConnection(this, mit->second);
            }
        }

        // 如果说加入遍历了一圈之后发现没有一个是超过观测阈值的，vPairs还为空
        // 那么刚刚统计的最大观测次数和对应的关键帧就派上用场了，将它们放到vPairs里面
        if (vPairs.empty())
        {
            vPairs.push_back(make_pair(nmax, pKFmax));
            pKFmax->AddConnection(this, nmax);
        }

        // 对vPairs进行排序，默认升序
        sort(vPairs.begin(), vPairs.end());
        list<KeyFrame *> lKFs;
        list<int> lWs;
        // 依次遍历获取vPairs中的关键帧和观测次数，分别放到两个list中
        for (size_t i = 0; i < vPairs.size(); i++)
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        {
            unique_lock<mutex> lockCon(mMutexConnections);

            // mspConnectedKeyFrames = spConnectedKeyFrames;
            mConnectedKeyFrameWeights = KFcounter;  // 将临时变量KFcounter赋给成员变量
            // 根据刚刚遍历得到的关键帧list生成vector赋给成员变量
            mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end()); // 同理，赋给成员变量

            if (mbFirstConnection && mnId != 0)
            {
                mpParent = mvpOrderedConnectedKeyFrames.front();
                mpParent->AddChild(this);
                mbFirstConnection = false;
            }
        }
    }

    void KeyFrame::AddChild(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.insert(pKF);
    }

    void KeyFrame::EraseChild(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.erase(pKF);
    }

    void KeyFrame::ChangeParent(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mpParent = pKF;
        pKF->AddChild(this);
    }

    set<KeyFrame *> KeyFrame::GetChilds()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens;
    }

    KeyFrame *KeyFrame::GetParent()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mpParent;
    }

    bool KeyFrame::hasChild(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens.count(pKF);
    }

    void KeyFrame::AddLoopEdge(KeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mbNotErase = true;
        mspLoopEdges.insert(pKF);
    }

    set<KeyFrame *> KeyFrame::GetLoopEdges()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspLoopEdges;
    }

    void KeyFrame::SetNotErase()
    {
        unique_lock<mutex> lock(mMutexConnections);
        mbNotErase = true;
    }

    void KeyFrame::SetErase()
    {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mspLoopEdges.empty())
            {
                mbNotErase = false;
            }
        }

        if (mbToBeErased)
        {
            SetBadFlag();
        }
    }

    void KeyFrame::SetBadFlag()
    {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mnId == 0)
                return;
            else if (mbNotErase)
            {
                mbToBeErased = true;
                return;
            }
        }

        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
            mit->first->EraseConnection(this);

        for (size_t i = 0; i < mvpMapPoints.size(); i++)
            if (mvpMapPoints[i])
                mvpMapPoints[i]->EraseObservation(this);
        {
            unique_lock<mutex> lock(mMutexConnections);
            unique_lock<mutex> lock1(mMutexFeatures);

            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();

            // Update Spanning Tree
            set<KeyFrame *> sParentCandidates;
            sParentCandidates.insert(mpParent);

            // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
            // Include that children as new parent candidate for the rest
            while (!mspChildrens.empty())
            {
                bool bContinue = false;

                int max = -1;
                KeyFrame *pC;
                KeyFrame *pP;

                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(), send = mspChildrens.end(); sit != send; sit++)
                {
                    KeyFrame *pKF = *sit;
                    if (pKF->isBad())
                        continue;

                    // Check if a parent candidate is connected to the keyframe
                    vector<KeyFrame *> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for (size_t i = 0, iend = vpConnected.size(); i < iend; i++)
                    {
                        for (set<KeyFrame *>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end(); spcit != spcend; spcit++)
                        {
                            if (vpConnected[i]->mnId == (*spcit)->mnId)
                            {
                                int w = pKF->GetWeight(vpConnected[i]);
                                if (w > max)
                                {
                                    pC = pKF;
                                    pP = vpConnected[i];
                                    max = w;
                                    bContinue = true;
                                }
                            }
                        }
                    }
                }

                if (bContinue)
                {
                    pC->ChangeParent(pP);
                    sParentCandidates.insert(pC);
                    mspChildrens.erase(pC);
                }
                else
                    break;
            }

            // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
            if (!mspChildrens.empty())
                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++)
                {
                    (*sit)->ChangeParent(mpParent);
                }

            mpParent->EraseChild(this);
            mTcp = Tcw * mpParent->GetPoseInverse();
            mbBad = true;
        }

        mpMap->EraseKeyFrame(this);
        mpKeyFrameDB->erase(this);
    }

    bool KeyFrame::isBad()
    {
        // 这是一个线程锁，所以这个函数的理解就是如果这个线程锁成功了，就是好的，否则就是坏的
        unique_lock<mutex> lock(mMutexConnections);
        return mbBad;
    }

    void KeyFrame::EraseConnection(KeyFrame *pKF)
    {
        bool bUpdate = false;
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mConnectedKeyFrameWeights.count(pKF))
            {
                mConnectedKeyFrameWeights.erase(pKF);
                bUpdate = true;
            }
        }

        if (bUpdate)
            UpdateBestCovisibles();
    }

    vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= mnGridCols)
            return vIndices;

        const int nMaxCellX = min((int)mnGridCols - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= mnGridRows)
            return vIndices;

        const int nMaxCellY = min((int)mnGridRows - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool KeyFrame::IsInImage(const float &x, const float &y) const
    {
        // 传入一个像素坐标，判断该点是否在图像中，其实非常简单粗暴，就是范围判断
        return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
    }

    cv::Mat KeyFrame::UnprojectStereo(int i)
    {
        const float z = mvDepth[i];
        if (z > 0)
        {
            const float u = mvKeys[i].pt.x;
            const float v = mvKeys[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

            unique_lock<mutex> lock(mMutexPose);
            return Twc.rowRange(0, 3).colRange(0, 3) * x3Dc + Twc.rowRange(0, 3).col(3);
        }
        else
            return cv::Mat();
    }

    float KeyFrame::ComputeSceneMedianDepth(const int q)
    {
        vector<MapPoint *> vpMapPoints;
        cv::Mat Tcw_;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPose);
            vpMapPoints = mvpMapPoints;
            Tcw_ = Tcw.clone();
        }

        // 新建一个vector用于存放算出来的每个特征点的深度
        vector<float> vDepths;
        // N是KeyFrame的成员变量，
        vDepths.reserve(N);
        // ???
        cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
        Rcw2 = Rcw2.t();
        float zcw = Tcw_.at<float>(2, 3);   // T矩阵的最后一列的第三行，也就是平移的z分量
        // 对于N个特征点，每个特征点都遍历计算深度
        for (int i = 0; i < N; i++)
        {
            // 前面提到的并不是每个特征点都有深度，所以这里判断一下 （? 为什么不是每个特征点都有深度）
            if (mvpMapPoints[i])
            {
                // 计算特征点深度
                MapPoint *pMP = mvpMapPoints[i];
                cv::Mat x3Dw = pMP->GetWorldPos();
                // 这里其实只对z分量进行了计算
                float z = Rcw2.dot(x3Dw) + zcw;
                // 将计算好的z分量放到vector中
                vDepths.push_back(z);
            }
        }

        // 对计算出的深度从小到大排序
        sort(vDepths.begin(), vDepths.end());

        // 返回中位数对应的深度，所有个数除以2，就是中间的那个数
        // 作者在这里对2做了参数化，也可以取其它值，那含义就变成了占总数 1/q 的元素
        return vDepths[(vDepths.size() - 1) / q];
    }

} // namespace ORB_SLAM

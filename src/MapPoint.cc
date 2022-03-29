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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include <mutex>

namespace ORB_SLAM2
{

    long unsigned int MapPoint::nNextId = 0;
    mutex MapPoint::mGlobalMutex;
    // 进行了一堆初始化赋值的操作
    MapPoint::MapPoint(
        const cv::Mat &Pos,
        KeyFrame *pRefKF,
        Map *pMap) : mnFirstKFid(pRefKF->mnId),
                     mnFirstFrame(pRefKF->mnFrameId),
                     nObs(0),
                     mnTrackReferenceForFrame(0),
                     mnLastFrameSeen(0),
                     mnBALocalForKF(0),
                     mnFuseCandidateForKF(0),
                     mnLoopPointForKF(0),
                     mnCorrectedByKF(0),
                     mnCorrectedReference(0),
                     mnBAGlobalForKF(0),
                     mpRefKF(pRefKF),
                     mnVisible(1),
                     mnFound(1),
                     mbBad(false),
                     mpReplaced(static_cast<MapPoint *>(NULL)),
                     mfMinDistance(0),
                     mfMaxDistance(0),
                     mpMap(pMap)
    {
        // 在这里，调用了Mat类型的成员函数copyTo，将数据拷贝给了MapPoint的成员变量mWorldPos
        // 这里mWorldPos代表的是某个点在绝对坐标系下的坐标，换句话说就是世界坐标系下的坐标
        // 但如果还有印象，我们传入的是初始帧相机坐标系下的三维点坐标，这两个坐标的定义并不一致
        // 这里之所以这样，是因为在初始化的过程中，我们将初始帧的相机坐标系当成了世界坐标系，后续所有的坐标点都已这个坐标系为准
        // 正如前面所说的，我们获取的是初始帧相机坐标系下的坐标，又将初始帧相机坐标系当作世界坐标系
        // 所以，这里的赋值就理所应当了
        Pos.copyTo(mWorldPos);
        // 另外，还新建了一个3*1，全为0的列向量
        mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        // 与之前类似的，每调用一次这个构造函数就会自动加一作为该地图点的索引
        mnId = nNextId++;
    }

    MapPoint::MapPoint(
        const cv::Mat &Pos,
        Map *pMap,
        Frame *pFrame,
        const int &idxF) : mnFirstKFid(-1),
                           mnFirstFrame(pFrame->mnId),
                           nObs(0),
                           mnTrackReferenceForFrame(0),
                           mnLastFrameSeen(0),
                           mnBALocalForKF(0),
                           mnFuseCandidateForKF(0),
                           mnLoopPointForKF(0),
                           mnCorrectedByKF(0),
                           mnCorrectedReference(0),
                           mnBAGlobalForKF(0),
                           mpRefKF(static_cast<KeyFrame *>(NULL)),
                           mnVisible(1),
                           mnFound(1),
                           mbBad(false),
                           mpReplaced(NULL),
                           mpMap(pMap)
    {
        Pos.copyTo(mWorldPos);
        cv::Mat Ow = pFrame->GetCameraCenter();
        mNormalVector = mWorldPos - Ow;
        mNormalVector = mNormalVector / cv::norm(mNormalVector);

        cv::Mat PC = Pos - Ow;
        const float dist = cv::norm(PC);
        const int level = pFrame->mvKeysUn[idxF].octave;
        const float levelScaleFactor = pFrame->mvScaleFactors[level];
        const int nLevels = pFrame->mnScaleLevels;

        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

        pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
    }

    void MapPoint::SetWorldPos(const cv::Mat &Pos)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        // 相比于关键帧的赋值，地图点的赋值会见到那很多，直接将传入的数值拷贝给成员变量mWorldPos即可
        Pos.copyTo(mWorldPos);
    }

    cv::Mat MapPoint::GetWorldPos()
    {
        unique_lock<mutex> lock(mMutexPos);
        // 返回当前地图点在世界坐标系下的坐标，它是以Mat类型存储的，大小为3*1，对应x,y,z
        return mWorldPos.clone();
    }

    cv::Mat MapPoint::GetNormal()
    {
        unique_lock<mutex> lock(mMutexPos);
        return mNormalVector.clone();
    }

    KeyFrame *MapPoint::GetReferenceKeyFrame()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpRefKF;
    }

    void MapPoint::AddObservation(KeyFrame *pKF, size_t idx)
    {
        // 增加线程锁防止ID冲突
        unique_lock<mutex> lock(mMutexFeatures);
        // count函数，返回指定元素的个数
        // 这里的意思是说，如果观测中已经有了我们传入的关键帧就不重复添加了，直接return
        if (mObservations.count(pKF))
            return;
        // 如果观测中没有当前帧的话，就添加
        // 注意这里成员变量mObservations是一个map，所以它的赋值方式和vector也不一样
        // 关键帧作为键(key)，值(value)是该地图点在该关键帧特征点列表中的索引
        mObservations[pKF] = idx;

        // mvuRight它是一个vector，元素是float，注释说是negative value of monocular points
        // 它是针对双目的情形使用的，对于弹幕情况mvuRight里面的值全为-1
        // 它一开始是在Frame.cc的228行赋值的，关键帧KeyFrame中的mvuRight是在构造函数中直接拷贝的，对应Frame的mvuRight
        // 这行代码的意思是如果它的值大于0，观测个数加2，否则加1
        // 结合它的定义就可以理解了，对于双目情况，一次有左右两个观测，所以观测次数+2，
        // 对于单目情况，在一开始赋值的时候全为-1，所以这里观测次数+1
        if (pKF->mvuRight[idx] >= 0)
            nObs += 2;
        else
            nObs++;
    }

    void MapPoint::EraseObservation(KeyFrame *pKF)
    {
        bool bBad = false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if (mObservations.count(pKF))
            {
                int idx = mObservations[pKF];
                if (pKF->mvuRight[idx] >= 0)
                    nObs -= 2;
                else
                    nObs--;

                mObservations.erase(pKF);

                if (mpRefKF == pKF)
                    mpRefKF = mObservations.begin()->first;

                // If only 2 observations or less, discard point
                if (nObs <= 2)
                    bBad = true;
            }
        }

        if (bBad)
            SetBadFlag();
    }

    map<KeyFrame *, size_t> MapPoint::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        // 简单粗暴，直接返回地图点的成员变量mObservations
        return mObservations;
    }

    int MapPoint::Observations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        // 返回该地图点对应的观测个数
        return nObs;
    }

    void MapPoint::SetBadFlag()
    {
        map<KeyFrame *, size_t> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mbBad = true;
            obs = mObservations;
            mObservations.clear();
        }
        for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;
            pKF->EraseMapPointMatch(mit->second);
        }

        mpMap->EraseMapPoint(this);
    }

    MapPoint *MapPoint::GetReplaced()
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mpReplaced;
    }

    void MapPoint::Replace(MapPoint *pMP)
    {
        if (pMP->mnId == this->mnId)
            return;

        int nvisible, nfound;
        map<KeyFrame *, size_t> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            obs = mObservations;
            mObservations.clear();
            mbBad = true;
            nvisible = mnVisible;
            nfound = mnFound;
            mpReplaced = pMP;
        }

        for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
        {
            // Replace measurement in keyframe
            KeyFrame *pKF = mit->first;

            if (!pMP->IsInKeyFrame(pKF))
            {
                pKF->ReplaceMapPointMatch(mit->second, pMP);
                pMP->AddObservation(pKF, mit->second);
            }
            else
            {
                pKF->EraseMapPointMatch(mit->second);
            }
        }
        pMP->IncreaseFound(nfound);
        pMP->IncreaseVisible(nvisible);
        pMP->ComputeDistinctiveDescriptors();

        mpMap->EraseMapPoint(this);
    }

    bool MapPoint::isBad()
    {
        // 和之前的理解类似，如果这两个线程锁都OK了，则是好的，否则是坏的
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mbBad;
    }

    void MapPoint::IncreaseVisible(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible += n;
    }

    void MapPoint::IncreaseFound(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound += n;
    }

    float MapPoint::GetFoundRatio()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound) / mnVisible;
    }

    void MapPoint::ComputeDistinctiveDescriptors()
    {
        // 这个函数的用途其实很简单
        // 由于一个地图点很自然会被多个关键帧观测到，反映在关键帧上就是多个特征点
        // 既然是特征点就会存在特征的描述子
        // 那么同一个地图点存在多个特征描述子，该如何取舍或计算一个独特的地图点的描述子呢？
        // 这个函数就是用来解决这件事的

        // Retrieve all observed descriptors
        vector<cv::Mat> vDescriptors;

        map<KeyFrame *, size_t> observations;

        {
            unique_lock<mutex> lock1(mMutexFeatures);
            // MapPoint的好坏flag，如果说这个MapPoint是坏的直接返回
            if (mbBad)
                return;
            // 否则的话将Distinctive成员变量赋给临时的observation
            observations = mObservations;
        }

        // 如果观测为空直接返回，换句话说就是如果这个地图点没有一个对应的关键帧，就返回
        if (observations.empty())
            return;

        // 观测个数等于这个地图点相关联的关键帧个数，而某个关键帧能和地图点建立联系，就说明在那个关键帧中看到了这个地图点
        // 再解释地通俗一点就是这个关键帧中存在这个地图点的特征点，既然是特征点，在之前就对特征点进行了描述
        vDescriptors.reserve(observations.size());

        // 依次获取地图点在各个关键帧中的描述子
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            // 获取每一个关键帧
            KeyFrame *pKF = mit->first;

            // 如果这个关键帧不是bad的话就执行下面的代码
            if (!pKF->isBad())
                // 获取到这个关键帧中指定特征点的描述子并放到vDescriptors
                // 还记得前面说过的，MapPoint的构造函数里需要传入两个参数，一个就是对应的关键帧，另一个就是这个地图点在关键帧特征点列表中的索引
                // 在这里，这两个变量就都派上了用场
                vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
        }

        // 经过了上面的循环之后，vDescriptors应该已经有几个元素了
        // 不同元素代表地图点在不同关键帧中的描述子
        // 如果描述子为空直接返回
        if (vDescriptors.empty())
            return;

        // Compute distances between them
        // 我们的目标是从这些描述子中找出一个最好的描述子作为地图点的描述子
        // 将描述子的个数设为N
        const size_t N = vDescriptors.size();

        float Distances[N][N];
        // 依次遍历各描述子，计算描述子之间的距离
        for (size_t i = 0; i < N; i++)
        {
            // 第i个描述子于第i个描述子之间的距离是0，即对角线元素为0
            Distances[i][i] = 0;
            // j从i+1开始迭代，例如i=1的时候，j=2,3,4...
            for (size_t j = i + 1; j < N; j++)
            {
                // 调用ORBmatcher的函数计算描述子之距离，注意下面这里的索引变化规律，算的是谁和谁的距离
                // i到j、j到i的距离是相同的，这里只需计算一次；也即这是一个对称矩阵，主对角线元素全为0
                int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
                Distances[i][j] = distij;
                Distances[j][i] = distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        // 在获取了多个描述子之间的距离以后，我们必须找到一个评价指标来选取一个最具代表性的描述子作为最终地图点的描述子
        // 这里作者用的是中位数距离进行选择，选择距离其他描述子中位数距离最小的描述子作为结果
        // 当然个人觉得也可以用其它评价标准，如均值、方差等等
        // 例如依次求取每个描述子到各描述子距离的方差，同样可以反映变化情况
        // 方差越小说明该描述子到其它描述子的距离波动不大，比较均衡、稳定，能够反映整体的趋势和情况
        int BestMedian = INT_MAX; // 最好的中位数距离
        int BestIdx = 0;          // 最好的描述子所对应的索引
        for (size_t i = 0; i < N; i++)
        {
            // 这里Distances[i]代表DiStance的第i行
            // 所以这一行代码的意思就是将DiStance的第i行直接赋给vDist向量
            // 向量的每个元素就表示第i个描述子到其它各个描述子之间的距离
            // 例如0-0，0-1，0-2...
            vector<int> vDists(Distances[i], Distances[i] + N);
            // 获得了这个距离后对其按从小到大排个序
            sort(vDists.begin(), vDists.end());
            // 然后直接取排完序后的序列中的中位数对应的值（其实就是第i个描述子到其它各描述子之间的距离）作为median
            int median = vDists[0.5 * (N - 1)];

            // 这里我们的原则是取到其它描述子中位数距离最小的作为结果，所以进行以下判断
            if (median < BestMedian)
            {
                BestMedian = median;
                BestIdx = i;
            }
        }

        // 经过一遍循环，我们可以依次求出每个描述子到其它描述子距离的中位数
        {
            // 最后一步，将我们挑选出来的最符合我们标准的描述子赋给MapPoint的成员变量mDescriptor，大功告成
            unique_lock<mutex> lock(mMutexFeatures);
            mDescriptor = vDescriptors[BestIdx].clone();
        }
    }

    cv::Mat MapPoint::GetDescriptor()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mDescriptor.clone();
    }

    int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));
    }

    void MapPoint::UpdateNormalAndDepth()
    {
        // 还是之前说的，一个observation是一个KeyFrame类+这个地图点在KeyFrame中对应特征点的索引
        map<KeyFrame *, size_t> observations;
        KeyFrame *pRefKF;
        cv::Mat Pos;
        // 线程同步锁
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            if (mbBad)
                return;
            // 如果一切OK的话就进行一些赋值操作
            observations = mObservations;
            pRefKF = mpRefKF;
            Pos = mWorldPos.clone();
        }

        // 如果观测为空，该地图点在这些观测中一次都没有出现，则直接返回
        if (observations.empty())
            return;

        // normal是一个3*1的全为0的向量
        cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
        int n = 0;
        // 在可观测到这个地图点的所有关键帧中迭代
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            // mit的第一个部分是关键帧KeyFrame
            KeyFrame *pKF = mit->first;
            // 获取这个关键帧的相机中心，是一个Mat类对象
            cv::Mat Owi = pKF->GetCameraCenter();
            // MapPoint世界坐标系下的坐标减去某个关键帧相机坐标系原点在世界坐标系下的坐标
            cv::Mat normali = mWorldPos - Owi;
            // 累加，归一化向量，为什么要这样做有待研究
            normal = normal + normali / cv::norm(normali);
            n++;
        }

        // 世界坐标系-参考帧相机中心的坐标
        cv::Mat PC = Pos - pRefKF->GetCameraCenter();
        // 对PC这个向量求范数，默认是L2范数，也就是欧氏距离
        const float dist = cv::norm(PC);
        // observation是一个map，我们根据KeyFrame可以找到它所对应的特征点索引
        // 在获得索引之后，这些特征点都是保存在KeyFrame的成员变量mvKeysUn里的，所以直接用这个索引就可以获得想要的特征点
        // 这个特征点又有成员变量octave，表示它是在金字塔的第几层提取出来的
        const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
        // 获得了该特征点对应的金字塔层数后，就可以进一步获得该层对应的尺度因子
        const float levelScaleFactor = pRefKF->mvScaleFactors[level];
        const int nLevels = pRefKF->mnScaleLevels;

        {
            unique_lock<mutex> lock3(mMutexPos);
            // 将距离（长度）乘以刚刚获得的尺度因子，就可以得到距离
            // 关于这两个参数的意义还有待进一步理解
            mfMaxDistance = dist * levelScaleFactor;
            mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
            mNormalVector = normal / n;
        }
    }

    float MapPoint::GetMinDistanceInvariance()
    {
        unique_lock<mutex> lock(mMutexPos);
        return 0.8f * mfMinDistance;
    }

    float MapPoint::GetMaxDistanceInvariance()
    {
        unique_lock<mutex> lock(mMutexPos);
        return 1.2f * mfMaxDistance;
    }

    int MapPoint::PredictScale(const float &currentDist, KeyFrame *pKF)
    {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pKF->mnScaleLevels)
            nScale = pKF->mnScaleLevels - 1;

        return nScale;
    }

    int MapPoint::PredictScale(const float &currentDist, Frame *pF)
    {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pF->mnScaleLevels)
            nScale = pF->mnScaleLevels - 1;

        return nScale;
    }

} // namespace ORB_SLAM

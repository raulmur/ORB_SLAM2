//
// Created by lan on 17-12-20.
//

#include "MapLine.h"

#include <mutex>
#include <include/LSDmatcher.h>
#include <map>

namespace ORB_SLAM2
{
    mutex MapLine::mGlobalMutex;
    long unsigned int MapLine::nNextId=0;
    long unsigned int MapLine::nNextCornerId=0;

MapLine::MapLine(Vector6d &Pos, KeyFrame *pRefKF, Map *pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopLineForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
{
    mWorldPos = Pos;

    mNormalVector << 0, 0, 0;

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;
}

    MapLine::MapLine(Vector6d &Pos, KeyFrame* pRefKF, Map* pMap,bool isCornerFrame):
            mnFirstKFid(pRefKF->mnCornerId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopLineForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
    {
        mWorldPos = Pos;

        mNormalVector << 0, 0, 0;

        // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnCornerId = nNextCornerId++;
    }

MapLine::MapLine(Vector6d &Pos, Map *pMap, Frame *pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopLineForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    mWorldPos = Pos;
    Mat Ow = pFrame->GetCameraCenter(); //相机光心坐标
    Vector3d OW;
    OW << Ow.at<double>(0), Ow.at<double>(1), Ow.at<double>(2);
    mStart3D = Pos.head(3);
    mEnd3D = Pos.tail(3);
    Vector3d midPoint = 0.5*(mStart3D + mEnd3D);
    mNormalVector = midPoint - OW; //世界坐标系下，相机到线段中点的向量
//    mNormalVector = mNormalVector/mNormalVector.norm();  //世界坐标系下，相机到3D点的单位向量
    mNormalVector.normalize();

    Vector3d PC = midPoint - OW;
    const float dist = PC.norm();   //相机光心到线段中点的距离

    const int level = pFrame->mvKeylinesUn[idxF].octave;
    const float levelScaleFactor = pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mLdesc.row(idxF).copyTo(mLDescriptor);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
    mnId = nNextId++;
}


    void MapLine::SetWorldPos(const Vector6d &Pos)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        mWorldPos = Pos;
    }

    Vector6d MapLine::GetWorldPos()
    {
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos;
    }

    Vector3d MapLine::GetNormal()
    {
        unique_lock<mutex> lock(mMutexPos);
        return mNormalVector;
    }

    KeyFrame* MapLine::GetReferenceKeyFrame()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpRefKF;
    }

    void MapLine::AddObservation(KeyFrame *pKF, size_t idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return;
        //记录下能观测到该MapLine的KF和该MapPoint在KF中的索引
        mObservations[pKF]=idx;

        nObs++;     //单目
    }

    void MapLine::EraseObservation(KeyFrame *pKF)
    {
        bool bBad = false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if(mObservations.count(pKF))
            {
                int idx = mObservations[pKF];
                if(pKF->mvuRight[idx]>=0)
                    nObs-=2;
                else
                    nObs--;

                mObservations.erase(pKF);

                // 如果该keyFrame是参考帧，该Frame被删除后重新指定RefFrame
                if(mpRefKF==pKF)
                    mpRefKF=mObservations.begin()->first;

                // 当观测到该特征线的相机数目少于2时，丢弃该点
                if(nObs<=2)
                    bBad=true;
            }
        }

        if(bBad)
            SetBadFlag();
    }

    map<KeyFrame*, size_t> MapLine::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    int MapLine::Observations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    int MapLine::GetIndexInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    bool MapLine::IsInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));
    }

    //告知可以观测到该MapLine的Frame,该MapLine已经被删除
    void MapLine::SetBadFlag()
    {
        map<KeyFrame*, size_t> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mbBad=true;
            obs = mObservations;    //把mObservations转存到obs，obs和mObservations里存的是指针，赋值过程为浅拷贝
            mObservations.clear();  //把mObservations指向的内存释放，obs作为局部变量之后自动删除
        }

        for(map<KeyFrame*, size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;
            pKF->EraseMapLineMatch(mit->second);    //告诉可以观测到该MapLine的KeyFrame，该MapLine被删除了
        }

        mpMap->EraseMapLine(this);  //擦除该MapLine申请的内存
    }

    //没有经过MapLineCulling检测的MapLines
    bool MapLine::isBad()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mbBad;
    }

    void MapLine::Replace(MapLine *pML)
    {
        if(pML->mnId==this->mnId)
            return;

        int nvisible, nfound;
        map<KeyFrame*, size_t> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            obs=mObservations;
            mObservations.clear();
            mbBad=true;
            nvisible=mnVisible;
            nfound = mnFound;
            mpReplaced = pML;
        }

        // 所有能观测到该MapLine的keyframe都要替换
        for(map<KeyFrame*, size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;

            if(!pML->IsInKeyFrame(pKF))
            {
                pKF->ReplaceMapLineMatch(mit->second, pML);
                pML->AddObservation(pKF, mit->second);
            } else{
                pKF->EraseMapLineMatch(mit->second);
            }
        }

        pML->IncreaseFound(nfound);
        pML->IncreaseVisible(nvisible);
        pML->ComputeDistinctiveDescriptors();

        mpMap->EraseMapLine(this);
    }

    MapLine* MapLine::GetReplaced()
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mpReplaced;
    }

    void MapLine::IncreaseVisible(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible+=n;
    }

    void MapLine::IncreaseFound(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound+=n;
    }

    float MapLine::GetFoundRatio()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound/mnVisible);
    }

    void MapLine::ComputeDistinctiveDescriptors()
    {
        // Retrieve all observed descriptors
        vector<Mat> vDescriptors;

        map<KeyFrame*, size_t> observations;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            if(mbBad)
                return ;
            observations = mObservations;
        }

        if(observations.empty())
            return;

        vDescriptors.reserve(observations.size());

        // 遍历观测到3d特征线的所有关键帧，获得LBD描述子，并插入到vDescriptors中
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;

            if(!pKF->isBad())
                vDescriptors.push_back(pKF->mLineDescriptors.row(mit->second));
        }

        if(vDescriptors.empty())
            return;

        // 计算这些描述子两两之间的距离
        const size_t NL=vDescriptors.size();

        //float Distances[N][N]
        std::vector<std::vector<float>> Distances;
        Distances.resize(NL, vector<float>(NL, 0));
        for(size_t i=0; i<NL; i++)
        {
            Distances[i][i]=0;
            for(size_t j=0; j<NL; j++)
            {
//                int distij = LSDmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
                int distij = norm(vDescriptors[i], vDescriptors[j], NORM_HAMMING);
                Distances[i][j]=distij;
                Distances[j][i]=distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for(size_t i=0; i<NL; i++)
        {
            vector<int> vDists(Distances[i].begin(), Distances[i].end());
            sort(vDists.begin(), vDists.end());

            //获得中值
            int median = vDists[0.5*(NL-1)];

            //寻找最小的中值
            if(median<BestMedian)
            {
                BestMedian = median;
                BestIdx = i;
            }
        }
        {
            unique_lock<mutex> lock(mMutexFeatures);
            mLDescriptor = vDescriptors[BestIdx].clone();
        }

    }

    Mat MapLine::GetDescriptor()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mLDescriptor.clone();
    }

    void MapLine::UpdateAverageDir()
    {
        map<KeyFrame*, size_t> observations;
        KeyFrame* pRefKF;
        Vector6d Pos;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            if(mbBad)
                return;
            observations = mObservations;
            pRefKF = mpRefKF;
            Pos = mWorldPos;
        }

        if(observations.empty())
            return;

        Vector3d normal(0, 0, 0);
        int n=0;
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;
            Mat Owi = pKF->GetCameraCenter();
            Vector3d OWi(Owi.at<float>(0), Owi.at<float>(1), Owi.at<float>(2));
            Vector3d middlePos = 0.5*(mWorldPos.head(3)+mWorldPos.tail(3));
            Vector3d normali = middlePos - OWi;
            normal = normal + normali/normali.norm();
            n++;
        }

        cv::Mat SP = (Mat_<float>(3,1) << Pos(0), Pos(1), Pos(2));
        cv::Mat EP = (Mat_<float>(3,1) << Pos(3), Pos(4), Pos(5));
        cv::Mat MP = 0.5*(SP+EP);

        cv::Mat CM = MP - pRefKF->GetCameraCenter();
        const float dist = cv::norm(CM);
        const int level = pRefKF->mvKeyLines[observations[pRefKF]].octave;
        const float levelScaleFactor = pRefKF->mvScaleFactors[level];
        const int nLevels = pRefKF->mnScaleLevels;

        {
            unique_lock<mutex> lock3(mMutexPos);
            mfMaxDistance = dist*levelScaleFactor;
            mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
            mNormalVector = normal/n;
        }
    }

    float MapLine::GetMinDistanceInvariance()
    {
        unique_lock<mutex> lock(mMutexPos);
        return 0.8f*mfMinDistance;
    }

    float MapLine::GetMaxDistanceInvariance()
    {
        unique_lock<mutex> lock(mMutexPos);
        return 1.2f*mfMaxDistance;
    }

    int MapLine::PredictScale(const float &currentDist, const float &logScaleFactor)
    {
        float ratio;
        {
            unique_lock<mutex> lock3(mMutexPos);
            ratio = mfMaxDistance/currentDist;
        }

        return ceil(log(ratio)/logScaleFactor);
    }
}
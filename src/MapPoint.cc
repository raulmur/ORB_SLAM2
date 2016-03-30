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

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;
MapPoint::MapPoint():
    nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0),mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0)
 { 
    //mNormalVector = cv::Mat::zeros(3,1,CV_32F);
    //unique_lock<recursive_mutex> lock(mpMap->mMutexPointCreation);
    //mpMap = new Map();
    // mpRefKF = new KeyFrame();
 }
MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This recursive_mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

template<class Archive>
    void MapPoint::save(Archive & ar, const unsigned int version) const
    {
        unsigned int nItems;bool is_id = false, is_valid = false;;
        size_t t_size;
        long unsigned int t_nId;

        if (mbBad)
            return;

        ar & const_cast<long unsigned int &> (mnId );
        //cout << "[" << mnId << "]" ;
        ar & nNextId;
        ar & const_cast<long int &> (mnFirstKFid);
        ar & const_cast<long int &> (mnFirstFrame);
        ar & const_cast<int &> (nObs);
        ar & const_cast<float &> (mTrackProjX);
        ar & const_cast<float &> (mTrackProjY);
        ar & const_cast<float &> (mTrackProjXR);
        ar & const_cast<bool &> (mbTrackInView);
        ar & const_cast<int &> (mnTrackScaleLevel);
        ar & const_cast<float &> (mTrackViewCos);
        ar & const_cast<long unsigned int &> (mnTrackReferenceForFrame);
        ar & const_cast<long unsigned int &> (mnLastFrameSeen);
        ar & const_cast<long unsigned int &> (mnBALocalForKF);
        ar & const_cast<long unsigned int &> (mnFuseCandidateForKF);
        ar & const_cast<long unsigned int &> (mnLoopPointForKF);
        ar & const_cast<long unsigned int &> (mnCorrectedByKF);
        ar & const_cast<long unsigned int &> (mnCorrectedReference);

        ar & const_cast<cv::Mat &> (mPosGBA);
        ar & const_cast<long unsigned int &> (mnBAGlobalForKF);
        ar & const_cast<cv::Mat &> (mWorldPos);
        
        // Save each KF point id
        nItems = mObservations.size();
        ar & nItems;
                //cout << "{INFO}mvpMapPoints nItems -" << nItems << endl;
        
        for (std::map<KeyFrame*,size_t>::const_iterator it = mObservations.begin(); it != mObservations.end(); ++it) {        
            if (it->first == NULL)
            {
                cout << "{INFO}Map POint Save - Empty observation " << mnId << endl;

                is_id = false;
                ar & is_id;
                continue;
            }
            else
            {
                is_id = true;
                ar & is_id;
                t_nId =  it->first->mnId;
                ar & t_nId;
                t_size = it->second;
                ar & t_size;
            }
 
            
        }
        

        ar & const_cast<cv::Mat &> (mNormalVector);
        ar & const_cast<cv::Mat &> (mDescriptor);
        if (mpRefKF) {
            is_valid = true;
            ar & is_valid;
            ar & mpRefKF->mnId;
        }
        else
        {
            is_valid = false;
            ar & is_valid;
        }
            

        ar & const_cast<int &> (mnVisible);
        ar & const_cast<int &> (mnFound);
        ar & const_cast<bool &> (mbBad);
        ar & const_cast<float &> (mfMinDistance);
        ar & const_cast<float &> (mfMaxDistance);           
       
    }

    template<class Archive>
    void MapPoint::load(Archive & ar, const unsigned int version)
    {
        unsigned int nItems;bool is_id = false, is_valid = false;
        size_t t_size;

        long unsigned int t_nId;

        ar & const_cast<long unsigned int &> (mnId );
        //cout << "[" << mnId << "]" ;
        ar & nNextId;
        ar & const_cast<long int &> (mnFirstKFid);
        ar & const_cast<long int &> (mnFirstFrame);
        ar & const_cast<int &> (nObs);
        ar & const_cast<float &> (mTrackProjX);
        ar & const_cast<float &> (mTrackProjY);
        ar & const_cast<float &> (mTrackProjXR);
        ar & const_cast<bool &> (mbTrackInView);
        ar & const_cast<int &> (mnTrackScaleLevel);
        ar & const_cast<float &> (mTrackViewCos);
        ar & const_cast<long unsigned int &> (mnTrackReferenceForFrame);
        ar & const_cast<long unsigned int &> (mnLastFrameSeen);
        ar & const_cast<long unsigned int &> (mnBALocalForKF);
        ar & const_cast<long unsigned int &> (mnFuseCandidateForKF);
        ar & const_cast<long unsigned int &> (mnLoopPointForKF);
        ar & const_cast<long unsigned int &> (mnCorrectedByKF);
        ar & const_cast<long unsigned int &> (mnCorrectedReference);

        ar & const_cast<cv::Mat &> (mPosGBA);
        ar & const_cast<long unsigned int &> (mnBAGlobalForKF);
        ar & const_cast<cv::Mat &> (mWorldPos);
        
         // Load each map point id
        ar & nItems;
        //mObservations.resize(nItems);
        //mObservations_nId.resize(nItems);
        for (int i = 0; i < nItems; ++i) { 

            ar & is_id;
            if (is_id)
            {
                ar & t_nId;
                ar & t_size;
                mObservations_nId[t_nId] = t_size;
            }
            else
            {

            }
        }
        
        ar & const_cast<cv::Mat &> (mNormalVector);
        ar & const_cast<cv::Mat &> (mDescriptor);

        //mpRefKF = new KeyFrame();
        //Reference Keyframe
        ar & is_valid;
        if (is_valid)
        {
            ar & t_nId;
        }
        else
            t_nId = 0;
        
        mref_KfId_pair = std::make_pair(t_nId,is_valid);

      	ar & const_cast<int &> (mnVisible);
        ar & const_cast<int &> (mnFound);
        ar & const_cast<bool &> (mbBad);
        ar & const_cast<float &> (mfMinDistance);
        ar & const_cast<float &> (mfMaxDistance);         
    }


// Explicit template instantiation
template void MapPoint::save<boost::archive::binary_oarchive>(
    boost::archive::binary_oarchive &, 
    const unsigned int) const;
template void MapPoint::save<boost::archive::binary_iarchive>(
    boost::archive::binary_iarchive &, 
    const unsigned int) const;
template void MapPoint::load<boost::archive::binary_oarchive>(
    boost::archive::binary_oarchive &, 
    const unsigned int);
template void MapPoint::load<boost::archive::binary_iarchive>(
    boost::archive::binary_iarchive &, 
    const unsigned int);

void MapPoint::SetMap(Map* map)
{
    mpMap = map;   
}

void MapPoint::SetObservations(std::vector<KeyFrame*> spKeyFrames)
{
    
    long unsigned int id, kfRef_id; size_t size;    
    //cout << "KF" << mnId <<" valid indexes-" << endl;
    int j = 0; 
    bool found_reference = false;
    kfRef_id = mref_KfId_pair.first;
    bool is_ref_valid = mref_KfId_pair.second;


    for (map<long unsigned int, size_t>::iterator it = mObservations_nId.begin(); it != mObservations_nId.end(); j++,++it) {
        id = it->first;
        size = it->second;        
        {
            for(std::vector<KeyFrame*>::iterator mit=spKeyFrames.begin(); mit !=spKeyFrames.end(); mit++)
            {
                KeyFrame* pKf = *mit;
                //cout << "[" << pKf->mnId << "]";               
                if(id == pKf->mnId)
                {                    
                    //cout << "[" << id <<"]";                    
                    mObservations[pKf] = size;
                    //id = -1;
                    break;
                }
            }
            
        }

    }

    for(std::vector<KeyFrame*>::iterator mit=spKeyFrames.begin(); mit !=spKeyFrames.end(); mit++)
    {
       KeyFrame* pKf = *mit;
       if (is_ref_valid && kfRef_id == pKf->mnId )
       {
            // Set the refernce Keyframe
            mpRefKF = pKf;
            found_reference = true;
       }
   }

    if (!found_reference)
    {
            mpRefKF = static_cast<KeyFrame*>(NULL);
            cout << "refernce KF - " << kfRef_id << "is not found for mappoint " << mnId << endl;  
            // Dummy KF
            //mpRefKF = new KeyFrame();  
    }
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This recursive_mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
     unique_lock<mutex> lock(mMutexFeatures);
     return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
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

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
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
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
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
    if(mObservations.count(pKF))
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
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    } 

    if (!pRefKF)
    {
        
            return;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;


    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, const float &logScaleFactor)
{
    float ratio;
    {
        unique_lock<mutex> lock3(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    return ceil(log(ratio)/logScaleFactor);
}

} //namespace ORB_SLAM

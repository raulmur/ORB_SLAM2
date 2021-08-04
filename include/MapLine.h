//
// Created by lan on 17-12-20.
//

#ifndef ORB_SLAM2_MAPLINE_H
#define ORB_SLAM2_MAPLINE_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

#include "line_descriptor/descriptor_custom.hpp"
//#include <opencv2/line_descriptor/descriptor.hpp>
#include <opencv2/core/core.hpp>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <map>

//using namespace line_descriptor;
using namespace cv::line_descriptor;
using namespace Eigen;
using namespace cv;
namespace ORB_SLAM2
{
class KeyFrame;
class Map;
class Frame;

typedef Matrix<double,6,1> Vector6d;
class MapLine
{
public:
    // 类比PL-SLAM
    MapLine(int idx_, Vector6d line3D_, cv::Mat desc_, int kf_obs_, Vector3d obs_, Vector3d dir_, Vector4d pts_, double sigma2_=1.f);
    ~MapLine(){};

    void addMapLineObervation(cv::Mat desc_, int kf_obs_, Vector3d obs_, Vector3d dir_, Vector4d pts_, double sigma2_=1.f);


//    int idx;                //线特征索引
//    bool inlier;            //是否属于内点
//    bool local;             //是否属于局部地图特征
//    Vector6d line3D;        // 3D endpoints of the line segment, 线段的两个端点的3D坐标
//    Vector3d med_obs_dir;   //观察该特征线段的方向


    vector<Vector3d> obs_list;  //每个观察点的坐标，2D线方程，用sqrt(lx2+ly2)归一化
    vector<Vector4d> pts_list;  //线特征两个端点的坐标，一共是4个坐标点

    vector<int> kf_obs_list;    //观测到线特征的KF的ID
    vector<double> sigma_list;  //每个观测值的sigma尺度

    ///类比ORB-SLAM2
    MapLine(Vector6d &Pos, KeyFrame* pRefKF, Map* pMap);   //关键帧创建MapLine
    MapLine(Vector6d &Pos, KeyFrame* pRefKF, Map* pMap,bool isCornerFrame);   //关键帧创建MapLine for Corner
    MapLine(Vector6d &Pos, Map* pMap, Frame* pFrame, const int &idxF); //普通帧创建MapLine

    void SetWorldPos(const Vector6d &Pos);
    Vector6d GetWorldPos();

    Vector3d GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    map<KeyFrame*, size_t> GetObservations();
    int Observations();


    void AddObservation(KeyFrame* pKF, size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapLine* pML);
    MapLine* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();   //仿照orbslam，计算线特征最独特的描述子

    Mat GetDescriptor();

    void UpdateAverageDir();    //pl-slam和ORB-SLAM的类似，计算线特征的平均方向

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, const float &logScaleFactor);

public:
    long unsigned int mnId; //Global ID for MapLine
    static long unsigned int nNextId;
        long unsigned int mnCornerId; //Global ID for corner
        static long unsigned int nNextCornerId;
    const long int mnFirstKFid; //创建该MapLine的关键帧ID
    const long int mnFirstFrame;    //创建该MapLine的帧ID，每一个关键帧都有一个帧ID
    int nObs;

    // Variables used by the tracking
    float mTrackProjX1;
    float mTrackProjY1;
    float mTrackProjX2;
    float mTrackProjY2;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    // TrackLocalMap - SearchByProjection中决定是否对该特征线进行投影的变量
    // mbTrackInView==false的点有几种：
    // a 已经和当前帧经过匹配（TrackReferenceKeyFrame，TrackWithMotionModel）但在优化过程中认为是外点
    // b 已经和当前帧经过匹配且为内点，这类点也不需要再进行投影
    // c 不在当前相机视野中的点（即未通过isInFrustum判断）
    bool mbTrackInView;
    // TrackLocalMap - UpdateLocalLines中防止将MapLines重复添加至mvpLocalMapLines的标记
    long unsigned int mnTrackReferenceForFrame;
    // TrackLocalMap - SearchLocalLines中决定是否进行isInFrustum判断的变量
    // mnLastFrameSeen==mCurrentFrame.mnId的line有集中：
    // a.已经和当前帧经过匹配（TrackReferenceKeyFrame, TrackWithMotionModel)，但在优化过程中认为是外点
    // b.已经和当前帧经过匹配且为内点，这类line也不需要再进行投影
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopLineForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;

    static std::mutex mGlobalMutex;

public:
    // Position in absolute coordinates
//    Mat mWorldPos;  //MapLine的中点在世界坐标系下的坐标
    Vector6d mWorldPos;
    Vector3d mStart3D;
    Vector3d mEnd3D;

    // KeyFrames observing the line and associated index in keyframe
    map<KeyFrame*, size_t> mObservations;   //观测到该MapLine的KF和该MapLine在KF中的索引

    Vector3d mNormalVector;  //MapPoint中，指的是该MapPoint的平均观测方向，这里指的是观测特征线段的方向

    cv::Mat mLDescriptor;   //通过ComputeDistinctiveDescriptors()得到的最优描述子

    KeyFrame* mpRefKF;  //参考关键帧

    vector<cv::Mat> mvDesc_list;  //线特征的描述子集
    vector<Vector3d> mvdir_list;  //每个观测线段的单位方向向量，中点

    //Tracking counters
    int mnVisible;
    int mnFound;

    // Bad flag , we don't currently erase MapPoint from memory
    bool mbBad;
    MapLine* mpReplaced;

    float mfMinDistance;
    float mfMaxDistance;

    Map* mpMap;

    mutex mMutexPos;
    mutex mMutexFeatures;
};

}


#endif //ORB_SLAM2_MAPLINE_H

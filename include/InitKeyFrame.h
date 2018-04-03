#ifndef INITKEYFRAME_H
#define INITKEYFRAME_H

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "SystemSetting.h"
#include <opencv2/opencv.hpp>
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
//#include "MapPoints.h"

namespace ORB_SLAM2
{

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class SystemSetting;
class KeyFrameDatabase;
//class ORBVocabulary;

class InitKeyFrame
{
public:    
    InitKeyFrame(SystemSetting &SS);

    void UndistortKeyPoints();
    bool PosInGrid(const cv::KeyPoint& kp, int &posX, int &posY);
    void AssignFeaturesToGrid();

public:

    ORBVocabulary* pVocabulary;
    //KeyFrameDatabase* pKeyFrameDatabase;

    long unsigned int nId;
    double TimeStamp;

    float fGridElementWidthInv;
    float fGridElementHeightInv;
    std::vector<std::size_t> vGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    float fx;
    float fy;
    float cx;
    float cy;
    float invfx;
    float invfy;
    float bf;
    float b;
    float ThDepth;
    int N;
    std::vector<cv::KeyPoint> vKps;
    std::vector<cv::KeyPoint> vKpsUn;
    cv::Mat Descriptors;

    //it's zero for mono
    std::vector<float> vRight;
    std::vector<float> vDepth;

    DBoW2::BowVector BowVec;
    DBoW2::FeatureVector FeatVec;

    int nScaleLevels;
    float fScaleFactor;
    float fLogScaleFactor;
    std::vector<float> vScaleFactors;
    std::vector<float> vLevelSigma2;
    std::vector<float> vInvLevelSigma2;
    std::vector<float> vInvScaleFactors;

    int nMinX;
    int nMinY;
    int nMaxX;
    int nMaxY;
    cv::Mat K;
    cv::Mat DistCoef;    

};

} //namespace ORB_SLAM2
#endif //INITKEYFRAME_H
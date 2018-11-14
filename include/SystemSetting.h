#ifndef SYSTEMSETTING_H
#define SYSTEMSETTING_H

#include <string>
#include "ORBVocabulary.h"
#include<opencv2/core/core.hpp>

namespace ORB_SLAM2 {

    class SystemSetting{

        //Load camera parameters from setting file
    public:

        SystemSetting(ORBVocabulary* pVoc);
        //SystemSetting::SystemSetting(ORBVocabulary* pVoc, KeyFrameDatabase* pKFDB );

        bool LoadSystemSetting(const std::string strSettingPath);

    public:
        //The Vocabulary and KeyFrameDatabase
        ORBVocabulary* pVocabulary;
        //KeyFrameDatabase* pKeyFrameDatabase;


        //Camera parameters
        float width;
        float height;
        float fx;
        float fy;
        float cx;
        float cy;
        float invfx;
        float invfy;
        float bf;
        float b;
        float fps;
        cv::Mat K;
        cv::Mat DistCoef;
        bool initialized;

        //Camera RGB parameters
        int nRGB;

        //ORB feature parameters
        int nFeatures;
        float fScaleFactor;
        int nLevels;
        float fIniThFAST;
        float fMinThFAST;

        //other parameters
        float ThDepth = -1;
        float DepthMapFactor = -1;

    };

} //namespace ORB_SLAM2

#endif //SystemSetting
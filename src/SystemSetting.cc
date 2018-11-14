#include <iostream>

#include "SystemSetting.h"

using namespace std;

namespace ORB_SLAM2 {

    SystemSetting::SystemSetting(ORBVocabulary* pVoc):
        pVocabulary(pVoc)
        {
        }

    //SystemSetting::SystemSetting(ORBVocabulary* pVoc, KeyFrameDatabase* pKFDB):
    //    pVocabulary(pVoc), pKeyFrameDatabase(pKFDB)
    //    {
    //    }


    bool SystemSetting::LoadSystemSetting(const std::string strSettingPath){
        cout<<endl<<"Loading System Parameters form:"<<strSettingPath<<endl;
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        width  = fSettings["Camera.width"];
        height = fSettings["Camera.height"];
        fx     = fSettings["Camera.fx"];
        fy     = fSettings["Camera.fy"];
        cx     = fSettings["Camera.cx"];
        cy     = fSettings["Camera.cy"];

        cv::Mat tmpK = cv::Mat::eye(3,3,CV_32F);
        tmpK.at<float>(0,0) = fx;
        tmpK.at<float>(1,1) = fy;
        tmpK.at<float>(0,2) = cx;
        tmpK.at<float>(1,2) = cy;
        tmpK.copyTo(K);

        cv::Mat tmpDistCoef(4,1,CV_32F);
        tmpDistCoef.at<float>(0) = fSettings["Camera.k1"];
        tmpDistCoef.at<float>(1) = fSettings["Camera.k2"];
        tmpDistCoef.at<float>(2) = fSettings["Camera.p1"];
        tmpDistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if( k3!=0 )
        {
            tmpDistCoef.resize(5);
            tmpDistCoef.at<float>(4) = k3;
        }
        tmpDistCoef.copyTo( DistCoef );

        bf = fSettings["Camera.bf"];
        fps= fSettings["Camera.fps"];

        invfx = 1.0f/fx;
        invfy = 1.0f/fy;
        b     = bf  /fx;
        initialized = true;

        cout<<"- size:"<<width<<"x"<<height<<endl;
        cout<<"- fx:"  <<fx<<endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if(DistCoef.rows==5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;
        cout << "- bf: " << bf << endl;

        //Load RGB parameter
        nRGB = fSettings["Camera.RGB"];

        //Load ORB feature parameters
        nFeatures = fSettings["ORBextractor.nFeatures"];
        fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        nLevels = fSettings["ORBextractor.nLevels"];
        fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        fMinThFAST = fSettings["ORBextractor.minThFAST"];

        cout << endl  << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        //Load others parameters, if the sensor is MONOCULAR, the parameters is zero;
        //ThDepth = fSettings["ThDepth"];
        //DepthMapFactor = fSettings["DepthMapFactor"];
        fSettings.release();
        return true;
    }

}
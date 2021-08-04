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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>


#include "ExtractLineSegment.h"
#include "MapLine.h"
#include "Lineextractor.h"
#include "auxiliar.h"
#include <fstream>
namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

    class MapPoint;
    class KeyFrame;
    class MapLine;
    class Frame
    {
    public:

        //Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc,/*提取直线特征*/ cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
        void ExtractLSD(int flag,const cv::Mat &imGray,const cv::Mat &imDepth);
        void isLineGood(const cv::Mat &imGray, const cv::Mat &imDepth);
        void isLineGood(const cv::Mat &imGray,  const cv::Mat &imDepth,cv::Mat K);
        vector<cv::Point3d> mv3DLineforMap;
        //使用LSD提取的直线
        //ntuple_list mDetectedLines;
        /**
         * 还需要哪些参数，
         * 描述子，数量，
         * */

        vector<FrameLine> mvframeLine;//have the same size with 3D lines
        vector<FrameLine> vAll2Dlines;//all the 2D lines in this Frame
        vector<Vector6d > mvLines3D;
        vector<FrameLine>	mVF3DLines;
        cv::Mat mRcwL;
        cv::Mat mtcwL;
        cv::Mat mRwcL;
        cv::Mat mOwL; //==mtwc

    public:
        Frame();

        // Copy constructor.
        Frame(const Frame &frame);

        // Constructor for stereo cameras.
        Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Constructor for RGB-D cameras.
        Frame(const cv::Mat &imGray,const double &timeStamp, const cv::Mat &imDepth,  ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Constructor for Monocular.
        Frame(const cv::Mat &imGray,  const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Constructor for Monocular with normal cameras.
        Frame(const cv::Mat &imGray, const cv::Mat &imNormal, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Extract ORB on the image. 0 for left image and 1 for right image.
        void ExtractORB(int flag, const cv::Mat &im);

        // extract line feature, 自己添加的
        void ExtractLSD(const cv::Mat &im);

        //点线特征选择
        void featureSelect(const cv::Mat &im);
        // 计算线特征端点的3D坐标，自己添加的
        void ComputeLine3D(Frame &frame1, Frame &frame2);

        // 自己添加的，线特征描述子MAD
        void lineDescriptorMAD( vector<vector<DMatch>> matches, double &nn_mad, double &nn12_mad) const;

        // Compute Bag of Words representation.
        void ComputeBoW();

        // Set the camera pose.
        void SetPose(cv::Mat Tcw);

        // Computes rotation, translation and camera center matrices from the camera pose.
        void UpdatePoseMatrices();

        // Returns the camera center.
        inline cv::Mat GetCameraCenter(){
            return mOw.clone();
        }

        // Returns inverse of rotation
        inline cv::Mat GetRotationInverse(){
            return mRwc.clone();
        }

        // Check if a MapPoint is in the frustum of the camera
        // and fill variables of the MapPoint to be used by the tracking
        bool isInFrustum(MapPoint* pMP, float viewingCosLimit);
        bool isInFrustum(MapLine* pML, float viewingCosLimit);
        // Compute the cell of a keypoint (return false if outside the grid)
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

        vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;
        vector<size_t> GetLinesInArea(const float &x1, const float &y1, const float &x2, const float &y2,
                                      const float &r, const int minLevel=-1, const int maxLevel=-1) const;

        // Search a match for each keypoint in the left image to a keypoint in the right image.
        // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
        void ComputeStereoMatches();

        // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
        void ComputeStereoFromRGBD(const cv::Mat &imDepth);
        void AnalysisSurfaceNormalfromImage(const cv::Mat &imNormal, cv::Mat &K);

        // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
        cv::Mat UnprojectStereo(const int &i);
        Vector6d obtain3DLine(const int &i);
    public:
        // Vocabulary used for relocalization.
        ORBVocabulary* mpORBvocabulary;

        // Feature extractor. The right is used only in the stereo case.
        ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
        // line feature extractor, 自己添加的
        LineSegment* mpLineSegment;
        // Frame timestamp.
        double mTimeStamp;

        // Calibration matrix and OpenCV distortion parameters.
        cv::Mat mK;
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx;
        static float invfy;
        cv::Mat mDistCoef;

        // Stereo baseline multiplied by fx.
        float mbf;

        // Stereo baseline in meters.
        float mb;

        // Threshold close/far points. Close points are inserted from 1 view.
        // Far points are inserted as in the monocular case from 2 views.
        float mThDepth;

        // Number of KeyPoints.
        int N;
        int NL; //特征线的个数
        bool dealWithLine;
        float blurNumber;
        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        // In the stereo case, mvKeysUn is redundant as images must be rectified.
        // In the RGB-D case, RGB images can be distorted.
        std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
        std::vector<cv::KeyPoint> mvKeysUn;

        // Corresponding stereo coordinate and depth for each keypoint.
        // "Monocular" keypoints have a negative value.
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;
        std::vector<float>  mvDepthLine;
        // Bag of Words Vector structures.
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // ORB descriptor, each row associated to a keypoint.
        cv::Mat mDescriptors, mDescriptorsRight;

        // MapPoints associated to keypoints, NULL pointer if no association.
        std::vector<MapPoint*> mvpMapPoints;

        // Flag to identify outlier associations.
        std::vector<bool> mvbOutlier;

        SurfaceNormal surfacenomal;
        vector<SurfaceNormal> vSurfaceNormal;
        std::vector<VanishingDirection> vVanishingDirection;
        vector<cv::Point2i> vSurfaceNormalx;vector<cv::Point2i> vSurfaceNormaly;vector<cv::Point2i> vSurfaceNormalz;

        vector<cv::Point3f> vSurfacePointx;vector<cv::Point3f> vSurfacePointy;vector<cv::Point3f> vSurfacePointz;

        std::vector<vector<cv::Point2d>> vVanishingLinex;std::vector<vector<cv::Point2d>> vVanishingLiney;std::vector<vector<cv::Point2d>> vVanishingLinez;
        //2D endpoints, 3D LinesPC
        std::vector<RandomPoint3d> vVaishingLinePCx;vector<RandomPoint3d> vVaishingLinePCy;vector<RandomPoint3d> vVaishingLinePCz;
        // 自己添加的，特征线vector，特征线的描述子
        Mat mLdesc;
        vector<KeyLine> mvKeylinesUn;
        vector<Vector3d> mvKeyLineFunctions;    //特征线段所在直线的系数
        // 和KeyPoint类似，自己添加，标识特征线段是否属于外点
        vector<bool> mvbLineOutlier;
        std::vector<MapLine*> mvpMapLines;  //mvpMapLines与keylines相关联

        // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;
        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        // Camera pose.
        cv::Mat mTcw;

        // Current and Next Frame id.
        static long unsigned int nNextId;
        long unsigned int mnId;

        // Reference Keyframe.
        KeyFrame* mpReferenceKF;

        // Scale pyramid info.
        int mnScaleLevels;
        float mfScaleFactor;
        float mfLogScaleFactor;
        vector<float> mvScaleFactors;
        vector<float> mvInvScaleFactors;
        vector<float> mvLevelSigma2;
        vector<float> mvInvLevelSigma2;

        // Undistorted Image Bounds (computed once).
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        static bool mbInitialComputations;


    private:

        // Undistort keypoints given OpenCV distortion parameters.
        // Only for the RGB-D case. Stereo must be already rectified!
        // (called in the constructor).
        void UndistortKeyPoints();

        // Computes image bounds for the undistorted image (called in the constructor).
        void ComputeImageBounds(const cv::Mat &imLeft);

        // Assign keypoints to the grid for speed up feature matching (called in the constructor).
        void AssignFeaturesToGrid();

        // Rotation, translation and camera center
        cv::Mat mRcw;
        cv::Mat mtcw;
        cv::Mat mRwc;
        cv::Mat mOw; //==mtwc
    };

}// namespace ORB_SLAM

#endif // FRAME_H
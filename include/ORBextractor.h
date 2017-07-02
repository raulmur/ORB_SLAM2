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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

/** \brief Compute the ORB features and descriptors on an image, with distribution limits.
*
* ORB features are used for stereo matching, tracking, mapping and place recognition. They
* are fast to compute, robust to scale and rotation, camera and lighting effects.
*
* ORB features are dispersed on the image using an octree using different scale levels. We try
* to detect a minimum number of corners in each cell. At each cell FAST corners are extracted 
* imposing a minimum response. Firstly we impose iniThFAST threshold. If no corners are detected 
* we impose a lower value minThFAST. The thresholds may need adjusting for low contrast images.
*
* Mask is ignored in the current implementation.
*/
class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    /** \brief Constructor
    * \param nfeatures Target number of features to extract
    * \param scaleFactor Scale factor of FAST corner features
    * \param nlevels Number of scale levels
    * \param iniThFAST Initial response threshold
    * \param minThFAST Minimum response threshold
    */
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    /** \brief Compute the ORB features and descriptors on an image
    * @param img the image to compute the features and descriptors on
    * @param mask the mask to apply
    * @param keypoints the resulting keypoints
    * @param descriptors output descriptors of keypoints
    */
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);  

	/// Compute the ORB keypoints on an image ensuring a wide distribution of key points
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    /** Compute the ORB keypoints on an image with no distribution limits
    * @param image_pyramid the image pyramid to compute the features and descriptors on
    * @param mask_pyramid the masks to apply at every level
    * @param keypoints the resulting keypoints, clustered per level
    */
    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    std::vector<cv::Point> pattern;

    /// Target number of features to extract
    int nfeatures;
    /// Scale factor of FAST corner features
    double scaleFactor;
    /// Number of scale levels
    int nlevels;

    /// Initial response threshold
    int iniThFAST;
    /// Minimum response threshold
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif


/*
 * =====================================================================================
 *
 *       Filename:  ProbabilityMapping.cc
 *
 *    Description:
 *
 *        Version:  0.1
 *        Created:  01/21/2016 10:39:12 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Josh Tang, Rebecca Frederick
 *
 *        version: 1.0
 *        created: 8/9/2016
 *        Log: fix a lot of bug, Almost rewrite the code
 *
 *        author: He Yijia
 *
 * =====================================================================================
 */

#ifndef PROBABILITYMAPPING_H
#define PROBABILITYMAPPING_H

#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <numeric>
//#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <mutex>

#define covisN 7
#define sigmaI 20
#define lambdaG 15//8
#define lambdaL 80
#define lambdaTheta 45  // 45
#define lambdaN 3
#define histo_length 30
#define th_high 100
#define th_low 50
#define THETA 0.23
#define NNRATIO 0.6

#define NULL_DEPTH 999

namespace ORB_SLAM2 {
class KeyFrame;
class Map;
} 


namespace cv {
class Mat;
}

class ProbabilityMapping {
public:

	struct depthHo {
		float depth;
		float sigma;
                Eigen::Vector3f Pw; // point pose in world frame
                bool supported;
                depthHo():depth(0.0),sigma(0.0),supported(false),Pw(0.0, 0.0, 0.0){}
        };

        ProbabilityMapping(ORB_SLAM2::Map *pMap);

        void Run();
        // add some const depth point into key frame
        void TestSemiDenseViewer();

        /* * \brief void SemiDenseLoop(ORB_SLAM2::KeyFrame kf, depthHo**, std::vector<depthHo>*): return results of epipolar search (depth hypotheses) */
        void SemiDenseLoop();
        /* * \brief void stereo_search_constraints(): return min, max inverse depth */
        void StereoSearchConstraints(ORB_SLAM2::KeyFrame* kf, float* min_depth, float* max_depth);
	/* * \brief void epipolar_search(): return distribution of inverse depths/sigmas for each pixel */
        void EpipolarSearch(ORB_SLAM2::KeyFrame *kf1, ORB_SLAM2::KeyFrame *kf2, const int x, const int y, float pixel, float min_depth, float max_depth, depthHo *dh, cv::Mat F12, float &best_u, float &best_v,float th_pi);
        void GetSearchRange(float& umin, float& umax, int px, int py, float mind, float maxd, ORB_SLAM2::KeyFrame* kf, ORB_SLAM2::KeyFrame* kf2);
        /* * \brief void inverse_depth_hypothesis_fusion(const vector<depthHo> H, depthHo* dist):
	 * *         get the parameters of depth hypothesis distrubution from list of depth hypotheses */
        void InverseDepthHypothesisFusion(const std::vector<depthHo>& h, depthHo &dist);
	/* * \brief void intraKeyFrameDepthChecking(std::vector<std::vector<depthHo> > h, int imrows, int imcols): intra-keyframe depth-checking, smoothing, and growing. */
        void IntraKeyFrameDepthChecking(std::vector<std::vector<depthHo> >& ho, int imrows, int imcols);
        //  vector is low.  use depth_map and detph_sigma (cv::Mat)  to speed
        void IntraKeyFrameDepthChecking(cv::Mat& depth_map, cv::Mat& depth_sigma, const cv::Mat gradimg);
        /* * \brief void interKeyFrameDepthChecking(ORB_SLAM2::KeyFrame* currentKF, std::vector<std::vector<depthHo> > h, int imrows, int imcols):
         * *         inter-keyframe depth-checking, smoothing, and growing. */
        void InterKeyFrameDepthChecking(const cv::Mat& im, ORB_SLAM2::KeyFrame* currentKF, std::vector<std::vector<depthHo> >& h);
        void InterKeyFrameDepthChecking(ORB_SLAM2::KeyFrame* currentKf);
        void RequestFinish()
        {
            //unique_lock<mutex> lock(mMutexFinish);
            mbFinishRequested = true;
        }

        bool CheckFinish()
        {
            //unique_lock<mutex> lock(mMutexFinish);
            return mbFinishRequested;
        }

private:
        bool mbFinishRequested;
        ORB_SLAM2::Map* mpMap;
        void GetTR(ORB_SLAM2::KeyFrame* kf, cv::Mat* t, cv::Mat* r);
        void GetXp(const cv::Mat& K, int x, int y, cv::Mat* Xp);
        void GetParameterization(const cv::Mat& F12, const int x, const int y, float &a, float &b, float &c);
        void ComputeInvDepthHypothesis(ORB_SLAM2::KeyFrame* kf, ORB_SLAM2::KeyFrame *kf2, float ustar, float ustar_var, float a, float b, float c, depthHo *dh, int x, int y);
        void GetGradientMagAndOri(const cv::Mat& image, cv::Mat* gradx, cv::Mat* grady, cv::Mat* mag, cv::Mat* ori);
        void GetInPlaneRotation(ORB_SLAM2::KeyFrame* k1, ORB_SLAM2::KeyFrame* k2, float* th);
        void PixelNeighborSupport(std::vector<std::vector<depthHo> > H, int x, int y, std::vector<depthHo>& support);
        void PixelNeighborNeighborSupport(std::vector<std::vector<depthHo> > H, int px, int py, std::vector<std::vector<depthHo> >& support);
        void GetIntensityGradient_D(const cv::Mat& ImGrad, float a, float b, float c, int px, float* q);
        void GetPixelDepth(float uj, int px, int py, ORB_SLAM2::KeyFrame* kf, ORB_SLAM2::KeyFrame *kf2, float &p);
        void GetPixelDepth(float uj, float vj, int px, int py, ORB_SLAM2::KeyFrame* kf, ORB_SLAM2::KeyFrame* kf2, float &p, depthHo *dh);
        bool ChiTest(const depthHo& ha, const depthHo& hb, float* chi_val);
        bool ChiTest(const float& a, const float& b, const float sigma_a, float sigma_b);
        //void GetFusion(const std::vector<depthHo>& best_compatible_ho, depthHo* hypothesis, float* min_sigma);
        void GetFusion(const std::vector<std::pair <float,float> > supported, float& depth, float& sigma);
        void GetFusion(const std::vector<depthHo>& best_compatible_ho, depthHo& hypothesis, float* min_sigma);
        void Equation14(depthHo& dHjn, float& depthp, cv::Mat& xp, cv::Mat& rji, cv::Mat& tji, float* res);
        cv::Mat ComputeFundamental(ORB_SLAM2::KeyFrame *&pKF1, ORB_SLAM2::KeyFrame *&pKF2);
        cv::Mat GetSkewSymmetricMatrix(const cv::Mat &v);

protected:
            std::mutex mMutexSemiDense;
};

#endif

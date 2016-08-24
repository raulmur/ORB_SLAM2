/* * =====================================================================================
 *
 *       Filename:  ProbabilityMapping.h
 *
 *    Description:  Semi-dense Probability Mapping Module for ORB-SLAM
 *    inspired by Raul-Mur Artal's paper
 *
 *        Version:  0.01
 *        Created:  01/21/2016 03:48:26 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Authors: Josh Tang, Rebecca Frederick 
 *   Organization:  Unkei
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
#define lambdaG 8
#define lambdaL 80
#define lambdaTheta 45
#define lambdaN 3
#define histo_length 30
#define th_high 100
#define th_low 50
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
                Eigen::Vector3d Pw; // point pose in world frame
                bool supported;
                depthHo():depth(0),sigma(0),supported(false){}
        };

        ProbabilityMapping(ORB_SLAM2::Map *pMap);

        void Run();
        // add some const depth point into key frame
        void TestSemiDenseViewer();

        /* * \brief void first_loop(ORB_SLAM2::KeyFrame kf, depthHo**, std::vector<depthHo>*): return results of epipolar search (depth hypotheses) */
        void FirstLoop(ORB_SLAM2::KeyFrame *kf, std::vector<std::vector<depthHo> > &ho);
        /* * \brief void stereo_search_constraints(): return min, max inverse depth */
        void StereoSearchConstraints(ORB_SLAM2::KeyFrame* kf, float* min_depth, float* max_depth);
	/* * \brief void epipolar_search(): return distribution of inverse depths/sigmas for each pixel */
        void EpipolarSearch(ORB_SLAM2::KeyFrame *kf1, ORB_SLAM2::KeyFrame *kf2, int x, int y, float pixel, cv::Mat grad, float min_depth, float max_depth, depthHo *dh);
	/* * \brief void inverse_depth_hypothesis_fusion(const vector<depthHo> H, depthHo* dist): 
	 * *         get the parameters of depth hypothesis distrubution from list of depth hypotheses */
        void InverseDepthHypothesisFusion(const std::vector<depthHo>& h, depthHo* dist);
	/* * \brief void intraKeyFrameDepthChecking(std::vector<std::vector<depthHo> > h, int imrows, int imcols): intra-keyframe depth-checking, smoothing, and growing. */
        void IntraKeyFrameDepthChecking(std::vector<std::vector<depthHo> >& ho, int imrows, int imcols);
        /* * \brief void interKeyFrameDepthChecking(ORB_SLAM2::KeyFrame* currentKF, std::vector<std::vector<depthHo> > h, int imrows, int imcols):
         * *         inter-keyframe depth-checking, smoothing, and growing. */
        void InterKeyFrameDepthChecking(const cv::Mat& im, ORB_SLAM2::KeyFrame* currentKF, std::vector<std::vector<depthHo> >& h);

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
        void GetParameterization(const cv::Mat& F12, const int x, const int y, float* a, float* b, float* c);
        void ComputeInvDepthHypothesis(ORB_SLAM2::KeyFrame* kf, int pixel, float ustar, float ustar_var, float a, float b, float c, depthHo *dh);
        void GetGradientMagAndOri(const cv::Mat& image, cv::Mat* gradx, cv::Mat* grady, cv::Mat* mag, cv::Mat* ori, cv::Mat* really);
        void GetInPlaneRotation(ORB_SLAM2::KeyFrame* k1, ORB_SLAM2::KeyFrame* k2, float* th);
        void PixelNeighborSupport(std::vector<std::vector<depthHo> > H, int x, int y, std::vector<depthHo>& support);
        void PixelNeighborNeighborSupport(std::vector<std::vector<depthHo> > H, int px, int py, std::vector<std::vector<depthHo> >& support);
        void GetIntensityGradient_D(const cv::Mat& ImGrad, float a, float b, float c, int px, float* q);
        void GetPixelDepth(int px, int py, ORB_SLAM2::KeyFrame* kf, float &p);
        //void GetPixelDepth(const cv::Mat& Im, const cv::Mat& R, const cv::Mat& T, ORB_SLAM2::KeyFrame* kF, int u, float *p);
	bool ChiTest(const depthHo& ha, const depthHo& hb, float* chi_val);
	void GetFusion(const std::vector<depthHo>& best_compatible_ho, depthHo* hypothesis, float* min_sigma);
        void Equation14(depthHo& dHjn, float& depthp, cv::Mat& xp, cv::Mat& rji, cv::Mat& tji, float* res);
        cv::Mat ComputeFundamental(ORB_SLAM2::KeyFrame *&pKF1, ORB_SLAM2::KeyFrame *&pKF2);
        cv::Mat GetSkewSymmetricMatrix(const cv::Mat &v);

protected:
            std::mutex mMutexSemiDense;
};

#endif

//
// Created by yan on 18-8-2.
//

#ifndef LINESEGMENT_UTIL_H
#define LINESEGMENT_UTIL_H




#include <stdio.h>
#include <fstream>
#include<iostream>
#include <numeric>
//#include "../include/Image_ScrollBar.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cxcore.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include"levmar.h"

using namespace std;

#define EPS	(1e-10)
#define PI 3.1415


class SurfaceNormal
{
public:
    cv::Point3f normal;
    cv::Point3f cameraPosition;
    cv::Point2i FramePosition;
    SurfaceNormal(){}
};



typedef struct meanshiftResult
{
    cv::Mat R_cm_Rec;
    float s_j_density;
    int axis;
}ResultOfMS;



typedef struct meanshift2d
{
    cv::Point2d centerOfShift;
    float density;

} sMS;

typedef struct RandomPoint3ds
{
    cv::Point3d pos;
    double		xyz[3];
    double      W_sqrt[3]; // used for mah-dist from pt to ln
    cv::Mat		cov;
    cv::Mat		U, W; // cov = U*D*U.t, D = diag(W); W is vector
    double 		DU[9];
    double      dux[3];
    RandomPoint3ds(){}
    RandomPoint3ds(cv::Point3d _pos)
    {
        pos = _pos;
        xyz[0] = _pos.x;
        xyz[1] = _pos.y;
        xyz[2] = _pos.z;

        cov = cv::Mat::eye(3,3,CV_64F);
        U = cv::Mat::eye(3,3,CV_64F);
        W = cv::Mat::ones(3,1,CV_64F);
    }
    RandomPoint3ds(cv::Point3d _pos, cv::Mat _cov)
    {
        pos = _pos;
        xyz[0] = _pos.x;
        xyz[1] = _pos.y;
        xyz[2] = _pos.z;
        cov = _cov.clone();
        cv::SVD svd(cov);
        U = svd.u.clone();
        W = svd.w.clone();
        W_sqrt[0] = sqrt(svd.w.at<double>(0));
        W_sqrt[1] = sqrt(svd.w.at<double>(1));
        W_sqrt[2] = sqrt(svd.w.at<double>(2));

        cv::Mat D = (cv::Mat_<double>(3,3)<<1/W_sqrt[0], 0, 0,
                0, 1/W_sqrt[1], 0,
                0, 0, 1/W_sqrt[2]);
        cv::Mat du = D*U.t();
        DU[0] = du.at<double>(0,0); DU[1] = du.at<double>(0,1); DU[2] = du.at<double>(0,2);
        DU[3] = du.at<double>(1,0); DU[4] = du.at<double>(1,1); DU[5] = du.at<double>(1,2);
        DU[6] = du.at<double>(2,0); DU[7] = du.at<double>(2,1); DU[8] = du.at<double>(2,2);
        dux[0] = DU[0]*pos.x + DU[1]*pos.y + DU[2]*pos.z;
        dux[1] = DU[3]*pos.x + DU[4]*pos.y + DU[5]*pos.z;
        dux[2] = DU[6]*pos.x + DU[7]*pos.y + DU[8]*pos.z;

    }
}RandomPoint3d;

typedef struct RandomLine3ds
{

    vector<RandomPoint3d> pts;  //supporting collinear points
    cv::Point3d A, B;
    cv::Point3d director; //director
    cv::Point3d mid;//middle point between two end points
    cv::Mat covA, covB;
    RandomPoint3d rndA, rndB;
    cv::Point3d u, d; // following the representation of Zhang's paper 'determining motion from...'
    RandomLine3ds () {}
    RandomLine3ds (cv::Point3d _A, cv::Point3d _B, cv::Mat _covA, cv::Mat _covB)
    {
        A = _A;
        B = _B;
        covA = _covA.clone();
        covB = _covB.clone();
    }

}RandomLine3d;

struct Data_optimizeRelmotion
{
    vector<RandomLine3d>& a;
    vector<RandomLine3d>& b;
    Data_optimizeRelmotion(vector<RandomLine3d>&ina, vector<RandomLine3d>& inb):a(ina),b(inb){}
};

typedef struct FrameLines
// FrameLine represents a line segment detected from a rgb-d frame.
// It contains 2d image position (endpoints, line equation), and 3d info (if
// observable from depth image).
{

    cv::Point2d  p, q;				// image endpoints p and q
    cv::Mat		 l;					// 3-vector of image line equation,
    double		 lineEq2d[3];
    cv::Point3d direction;
    bool		 haveDepth;			// whether have depth
    //RandomLine3d line3d;
    vector<RandomPoint3d> rndpts3d;

    cv::Point2d	 r;					// image line gradient direction (polarity);
    cv::Mat		 des;				// image line descriptor;
//	double*		 desc;

    int			 lid;				// local id in frame
    int			 gid;				// global id;

    int			 lid_prvKfrm;		// correspondence's lid in previous keyframe

    FrameLines() {gid = -1;}
    FrameLines(cv::Point2d p_, cv::Point2d q_);

    cv::Point2d getGradient(cv::Mat* xGradient, cv::Mat* yGradient);
    void complineEq2d()
    {
        cv::Mat pt1 = (cv::Mat_<double>(3,1)<<p.x, p.y, 1);
        cv::Mat pt2 = (cv::Mat_<double>(3,1)<<q.x, q.y, 1);
        cv::Mat lnEq = pt1.cross(pt2); // lnEq = pt1 x pt2
        lnEq = lnEq/sqrt(lnEq.at<double>(0)*lnEq.at<double>(0)
                         +lnEq.at<double>(1)*lnEq.at<double>(1)); // normalize, optional
        lineEq2d[0] = lnEq.at<double>(0);
        lineEq2d[1] = lnEq.at<double>(1);
        lineEq2d[2] = lnEq.at<double>(2);

    }
} FrameLine;

class VanishingDirection
{
public:
    cv::Point3d direction;
    vector<RandomPoint3d> pts;
    cv::Point2d start2DPoint;
    cv::Point2d end2DPoint;
    VanishingDirection(){}
};
typedef struct axiSNVector{
    vector<SurfaceNormal> SNVector;
    vector<FrameLine>	Linesvector;
    int axis;
}axiSNV;
class MyTimer
{
#ifdef OS_WIN
    public:
	MyTimer() {	QueryPerformanceFrequency(&TickPerSec);	}

	LARGE_INTEGER TickPerSec;        // ticks per second
	LARGE_INTEGER Tstart, Tend;           // ticks

	double time_ms;
	double time_s;
	void start()  {	QueryPerformanceCounter(&Tstart);}
	void end() 	{
		QueryPerformanceCounter(&Tend);
		time_ms = (Tend.QuadPart-Tstart.QuadPart)*1000.0/TickPerSec.QuadPart;
		time_s = time_ms/1000.0;
	}
#else
public:
    timespec t0, t1;
    MyTimer() {}
    double time_ms;
    double time_s;
    void start() {
        clock_gettime(CLOCK_REALTIME, &t0);
    }
    void end() {
        clock_gettime(CLOCK_REALTIME, &t1);
        time_ms = t1.tv_sec * 1000 + t1.tv_nsec/1000000.0 - (t0.tv_sec * 1000 + t0.tv_nsec/1000000.0);
        time_s = time_ms/1000.0;
    }
#endif
};

template<class bidiiter> //Fisher-Yates shuffle
bidiiter random_unique(bidiiter begin, bidiiter end, size_t num_random) {
    size_t left = std::distance(begin, end);
    while (num_random--) {
        bidiiter r = begin;
        std::advance(r, rand()%left);
        std::swap(*begin, *r);
        ++begin;
        --left;
    }
    return begin;
}
/*
void SystemParameters::init()
{
    // ----- 2d-line -----
   float line_segment_len_thresh		= 10;		// pixels, min lenght of image line segment to use

    // ----- 3d-line measurement ----
    float ratio_of_collinear_pts		= 0.6;		// ratio, decide if a frameline has enough collinear pts
    float pt2line_dist_extractline	= 0.02;		// meter, threshold pt to line distance when detect lines from pts
   float pt2line_mahdist_extractline	= 1.5;		// NA,	  as above
   int ransac_iters_extract_line	= 100;		// 1, max ransac iters when detect lines from pts
    float num_cells_lineseg_range		= 10;		// 1,
    float ratio_support_pts_on_line	= 0.7;		// ratio, when verifying a detected 3d line
    float line3d_length_thresh		= 0.02;		// in meter, ignore too short 3d line segments

    // ----- camera model -----
    float stdev_sample_pt_imgline		= 1;		// in pixel, std dev of sample point from an image line
   float depth_stdev_coeff_c1		= 0.00273;	// c1,c2,c3: coefficients of depth noise quadratic function
   float depth_stdev_coeff_c2		= 0.00074;
   float depth_stdev_coeff_c3		= -0.00058;

    // ----- key frame -----
   float num_raw_frame_skip			= 1;
   float window_length_keyframe		= 1;
   float num_2dlinematch_keyframe	= 30;		// detect keyframe, minmum number of 2d line matches left
   float num_3dlinematch_keyframe	= 40;

    // ----- relative motion -----
   float pt2line3d_dist_relmotion	= 0.05;		// in meter,
   float line3d_angle_relmotion		= 10;
   float fast_motion					= 1;
   float inlier_ratio_constvel		= 0.4;
   bool dark_ligthing				= false;
   float max_img_brightness			= 0;

    // ----- lba -----
   float num_pos_lba	= 5;
float num_frm_lba	= 7;
bool g2o_BA_use_kernel			= true;
float g2o_BA_kernel_delta			= 10;

    // ----- loop closing -----
    float loopclose_interval			= 50;  // frames, check loop closure
float  loopclose_min_3dmatch		= 30;  // min_num for 3d line matches between two frames

    // ----- lsd setting -----
    float lsd_angle_th = 40;						// default: 22.5 deg
float lsd_density_th = 0.7;					// default: 0.7

*/


RandomLine3d extract3dline(vector<cv::Point3d>& pts,ofstream &origPoints, ofstream &optiPoints);
double dist3d_pt_line (cv::Point3f X, cv::Point3f A, cv::Point3f B);
double dist3d_pt_line (cv::Mat x, cv::Point3d A, cv::Point3d B);
bool verify3dLine(vector<cv::Point3d> pts, cv::Point3d A, cv::Point3d B);
bool verify3dLine(const vector<RandomPoint3d>& pts, const cv::Point3d& A,  const cv::Point3d& B);
void computeLine3d_svd (vector<cv::Point3d> pts, cv::Point3d& mean, cv::Point3d& drct);
void computeLine3d_svd (vector<RandomPoint3d> pts, cv::Point3d& mean, cv::Point3d& drct);
void computeLine3d_svd (const vector<RandomPoint3d>& pts, const vector<int>& idx, cv::Point3d& mean, cv::Point3d& drct);
cv::Point3d projectPt3d2Ln3d (const cv::Point3d& P, const cv::Point3d& mid, const cv::Point3d& drct);
cv::Mat array2mat(double a[], int n);
cv::Point3d mat2cvpt3d (cv::Mat m);
cv::Point2d mat2cvpt (const cv::Mat& m);
cv::Mat cvpt2mat(const cv::Point3d& p, bool homo);
cv::Mat cvpt2mat(cv::Point2d p, bool homo);
int computeMSLD (FrameLine& l, cv::Mat* xGradient, cv::Mat* yGradient);
int computeSubPSR (cv::Mat* xGradient, cv::Mat* yGradient,
                   cv::Point2d p, double s, cv::Point2d g, vector<double>& vs);
void MLEstimateLine3d_compact (RandomLine3d& line,	int maxIter);
void matchLine (vector<FrameLine> f1, vector<FrameLine> f2, vector<vector<int> >& matches);
void trackLine (vector<FrameLine> f1, vector<FrameLine> f2, vector<vector<int> >& matches);
vector<int> computeRelativeMotion_Ransac (vector<RandomLine3d> a, vector<RandomLine3d> b, cv::Mat& R, cv::Mat& t);
RandomLine3d extract3dline_mahdist(const vector<RandomPoint3d>& pts);
double mah_dist3d_pt_line (const RandomPoint3d& pt, const cv::Point3d& q1, const cv::Point3d& q2);
cv::Mat vec2SkewMat (cv::Mat vec);
cv::Mat vec2SkewMat (cv::Point3d vec);
void costFun_MLEstimateLine3d_compact(double *p, double *error, int m, int n, void *adata);
double closest_3dpt_ratio_online_mah (const RandomPoint3d& pt, cv::Point3d q1, cv::Point3d q2);
double lineSegmentOverlap(const FrameLine& a, const FrameLine& b);
double line_to_line_dist2d(FrameLine& a, FrameLine& b);
double projectPt2d_to_line2d(const cv::Point2d& X, const cv::Point2d& A, const cv::Point2d& B);
double pt_to_line_dist2d(const cv::Point2d& p, double l[3]);
bool compute_motion_given_ptpair_file (string filename, const cv::Mat& depth, cv::Mat& R, cv::Mat& t);
bool computeRelativeMotion_svd (vector<cv::Point3d> a, vector<cv::Point3d> b, cv::Mat& R, cv::Mat& t);
void computeRelativeMotion_svd (vector<RandomLine3d> a, vector<RandomLine3d> b, cv::Mat& R, cv::Mat& t);
void computeLine3d_svd (const vector<RandomPoint3d>& pts, const vector<int>& idx, cv::Point3d& mean, cv::Point3d& drct);

void optimizeRelmotion(vector<RandomLine3d> a, vector<RandomLine3d> b, cv::Mat& R, cv::Mat& t);
cv::Mat r2q(cv::Mat R);
cv::Mat q2r(cv::Mat q);
cv::Mat q2r (double* q);
void costFun_optimizeRelmotion(double *p, double *error, int m, int n, void *adata);
RandomPoint3d compPt3dCov (cv::Point3d pt, cv::Mat K, double);
double depthStdDev (double d);
#endif //LINESEGMENT_UTIL_H

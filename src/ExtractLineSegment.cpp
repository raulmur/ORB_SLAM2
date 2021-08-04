//
// Created by lan on 17-12-13.
//

//#include <Thirdparty/line_descriptor/include/line_descriptor_custom.hpp>
#include "ExtractLineSegment.h"
//#include <opencv2/line_descriptor/descriptor.hpp>
#include "line_descriptor/descriptor_custom.hpp"

namespace ORB_SLAM2
{
LineSegment::LineSegment() {}

void LineSegment::ExtractLineSegment(const Mat &img, vector<KeyLine> &keylines, Mat &ldesc, vector<Vector3d> &keylineFunctions, int scale, int numOctaves)
{
#if 1
    // detect line features
    Ptr<BinaryDescriptor> lbd = BinaryDescriptor::createBinaryDescriptor();
    Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();

    line_descriptor::LSDDetectorC::LSDOptions opts;
    opts.refine = 2;
    opts.scale = 1.2;
    opts.sigma_scale = 0.6;
    opts.quant = 2.0;
    opts.ang_th = 22.5;
    opts.log_eps = 1.0;
    opts.density_th = 0.6;
    opts.n_bins = 1024;
    opts.min_length = 0.025;

    lsd->detect(img, keylines, scale, 1, opts);
#endif
//    Ptr<BinaryDescriptor> lbd = BinaryDescriptor::createBinaryDescriptor();
//    Ptr<line_descriptor::LSDDetector> lsd = line_descriptor::LSDDetector::createLSDDetector();
    lsd->detect(img, keylines, scale, numOctaves);

    unsigned int lsdNFeatures = 40;

    // filter lines
    if(keylines.size()>lsdNFeatures)
    {
        sort(keylines.begin(), keylines.end(), sort_lines_by_response());
        keylines.resize(lsdNFeatures);
        for(unsigned int i=0; i<lsdNFeatures; i++)
            keylines[i].class_id = i;
    }

    lbd->compute(img, keylines, ldesc);     //计算特征线段的描述子

    // 计算特征线段所在直线的系数
    for(vector<KeyLine>::iterator it=keylines.begin(); it!=keylines.end(); ++it)
    {
        Vector3d sp_l; sp_l << it->startPointX, it->startPointY, 1.0;
        //cout<<it->startPointX<<","<<it->startPointY<<","<<1.0<<endl;
        Vector3d ep_l; ep_l << it->endPointX, it->endPointY, 1.0;
        //cout<<it->endPointX<<","<<it->endPointY<<","<<1.0<<endl;
        Vector3d lineF;     //直线方程
        lineF << sp_l.cross(ep_l);
        //cout<<lineF(0)<<","<<lineF(1)<<","<<lineF(2)<<endl;
        lineF = lineF / sqrt(lineF(0)*lineF(0)+lineF(1)*lineF(1));
        //lineF=lineF/sqrt(lineF(0)*lineF(0)+lineF(1)*lineF(1));
        keylineFunctions.push_back(lineF);
    }
}

void LineSegment::LineSegmentMatch(Mat &ldesc_1, Mat &ldesc_2)
{
    BFMatcher bfm(NORM_HAMMING, false);
    bfm.knnMatch(ldesc_1, ldesc_2, line_matches, 2);
}

void LineSegment::lineDescriptorMAD()
{
    vector<vector<DMatch>> matches_nn, matches_12;
    matches_nn = line_matches;
    matches_12 = line_matches;

    // estimate the NN's distance standard deviation
    double nn_dist_median;
    sort( matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist());
    nn_dist_median = matches_nn[int(matches_nn.size()/2)][0].distance;
    for(unsigned int i=0; i<matches_nn.size(); i++)
        matches_nn[i][0].distance = fabsf(matches_nn[i][0].distance - nn_dist_median);
    sort(matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist());
    nn_mad = 1.4826 * matches_nn[int(matches_nn.size()/2)][0].distance;

    // estimate the NN's 12 distance standard deviation
    double nn12_dist_median;
    sort( matches_12.begin(), matches_12.end(), conpare_descriptor_by_NN12_dist());
    nn12_dist_median = matches_12[int(matches_12.size()/2)][1].distance - matches_12[int(matches_12.size()/2)][0].distance;
    for (unsigned int j=0; j<matches_12.size(); j++)
        matches_12[j][0].distance = fabsf( matches_12[j][1].distance - matches_12[j][0].distance - nn12_dist_median);
    sort(matches_12.begin(), matches_12.end(), compare_descriptor_by_NN_dist());
    nn12_mad = 1.4826 * matches_12[int(matches_12.size()/2)][0].distance;
}

double LineSegment::lineSegmentOverlap(double spl_obs, double epl_obs, double spl_proj, double epl_proj)
{
    double sln    = min(spl_obs,  epl_obs);     //线特征两个端点的观察值的最小值
    double eln    = max(spl_obs,  epl_obs);     //线特征两个端点的观察值的最大值
    double spn    = min(spl_proj, epl_proj);    //线特征两个端点的投影点的最小值
    double epn    = max(spl_proj, epl_proj);    //线特征两个端点的投影点的最大值

    double length = eln-spn;

    double overlap;
    if ( (epn < sln) || (spn > eln) )
        overlap = 0.f;
    else{
        if ( (epn>eln) && (spn<sln) )
            overlap = eln-sln;
        else
            overlap = min(eln,epn) - max(sln,spn);
    }

    if(length>0.01f)
        overlap = overlap / length;
    else
        overlap = 0.f;

    return overlap;
}

}
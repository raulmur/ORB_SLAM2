//
// Created by yan on 18-8-2.
//

#include "Lineextractor.h"
//
// Created by yan on 18-8-2.
//

struct Data_MLEstimateLine3d_Compact
{
    int idx1, idx2;
    vector<RandomPoint3d>& pts;
    Data_MLEstimateLine3d_Compact(vector<RandomPoint3d>& _pts) :pts(_pts) {}

};

RandomLine3d extract3dline( vector<cv::Point3d>& pts,ofstream &origPoints,
                            ofstream &optiPoints)
// extract a single 3d line from point clouds using ransac
// input: 3d points
// output: inlier points, line parameters: midpt and direction
{

    int maxIterNo = 30;//sysPara.ransac_iters_extract_line;
    double distThresh = 0.02;//sysPara.pt2line_dist_extractline; // meter
    // distance threshold should be adapted to line length and depth
    int minSolSetSize = 2;

    vector<int> indexes(pts.size());


    for (int i=0; i<indexes.size(); ++i) indexes[i]=i;
    vector<cv::Point3d> maxInlierSet;

    cv::Point3d bestA, bestB;
    //一堆点中，找出来直线的两端点
    for(int iter=0; iter<maxIterNo;iter++) {
        vector<cv::Point3d> inlierSet;
        //洗牌算法，找不同的两个数 indexes1 and indexes2
        random_unique(indexes.begin(), indexes.end(),minSolSetSize);// shuffle
        cv::Point3d A = pts[indexes[0]], B = pts[indexes[1]];
        cout<<"A:"<<A.x<<","<<A.y<<","<<A.z<<".B:"<<B.x<<endl;

        // compute a line from A and B
        if (cv::norm(B-A) < EPS ) continue;

        /*
         * 随机取出来两点 作为直线端点，计算 其他点X 到直线距离，距离小于 阈值（disThresh）时，把X作为inlierSet,
         * 最后，size of inlierSet 最大的两个端点，最为bestA and bestB，点集则是maxInlierSet
         *
        */

        for (int i=0; i<pts.size(); ++i) {
            // compute distance to AB
            //	MyTimer t;
            //	t.start();
            double dist = dist3d_pt_line(pts[i],A,B);

            cout<<"dist takes "<<dist<<endl;
            if (dist<distThresh) {
                inlierSet.push_back(pts[i]);
            }
        }
        //cout<<"inlier"<<inlierSet<<endl;
        if(inlierSet.size() > maxInlierSet.size())	{
            if (verify3dLine(inlierSet, A, B)) {
                maxInlierSet = inlierSet;
                bestA = A; bestB = B;
            }
        }
    }

    //cout<<"初步筛选出一个点集：size of MaxInlierSet"<<maxInlierSet.size()<<endl;

    RandomLine3d rl;

    for(int i=0; i<maxInlierSet.size(); ++i)
        rl.pts.push_back(RandomPoint3ds(maxInlierSet[i]));

    cout<<"Lineextractor:point number of rl:"<<rl.pts.size()<<endl;

    if (maxInlierSet.size() >= 2) {
        cv::Point3d m = (bestA+bestB)*0.5, d = bestB-bestA;
        // optimize and reselect inliers
        // compute a 3d line using algebraic method
        while(true) {
            vector<cv::Point3d> tmpInlierSet;
            cv::Point3d tmp_m, tmp_d;
            //对点集进行精炼
            computeLine3d_svd(maxInlierSet, tmp_m, tmp_d);

            for(int i=0; i<pts.size(); ++i) {
                if(dist3d_pt_line(pts[i],tmp_m, tmp_m+tmp_d) < distThresh) {
                    tmpInlierSet.push_back(pts[i]);
                }
            }
            if(tmpInlierSet.size()>maxInlierSet.size()) {
                maxInlierSet = tmpInlierSet;
                m = tmp_m;
                d = tmp_d;
            } else
                break;
        }

        // find out two endpoints
        double minv=100, maxv=-100;
        int	   idx_end1 = 0, idx_end2 = 0;
        for(int i=0; i<maxInlierSet.size(); ++i) {
            double dproduct = (maxInlierSet[i]-m).dot(d);
            if ( dproduct < minv) {
                minv = dproduct;
                idx_end1 = i;
            }
            if (dproduct > maxv) {
                maxv = dproduct;
                idx_end2 = i;
            }
        }
        rl.A = maxInlierSet[idx_end1];
        rl.B = maxInlierSet[idx_end2];

        /*
         * 对其他的点进行修改
         *
         * */
        cout<<rl.A.x<<",y"<<rl.A.y<<",z"<<rl.A.z<<"; x="<<rl.B.x<<",y= "<<rl.B.y<<",z="<<rl.B.z<<endl;
        rl.director=rl.A-rl.B;
        rl.mid=(rl.A+rl.B)/2;
        //cout<<"director"<<director.x<<"-"<<rl.A.x-rl.B.x<<endl;
        //cout<<"point number opti"<<pts.size()<<endl;



        rl.pts.clear();
        for(int i=0; i<maxInlierSet.size(); ++i)
            rl.pts.push_back(RandomPoint3d(maxInlierSet[i]));
    }

    return rl;

}


double dist3d_pt_line (cv::Point3f X, cv::Point3f A, cv::Point3f B)
// input: point X, line (A,B)
{
    if(cv::norm(A-B)<EPS) {
        cerr<<"error in function dist3d_pt_line: line length can not be 0!"<<endl;
        return -1;
    }
    double ax = cv::norm(X-A);
    cv::Point3d nvAB = (B-A) * (1/cv::norm(A-B));
    return sqrt(abs( ax*ax - ((X-A).dot(nvAB))*((X-A).dot(nvAB))));
}



bool verify3dLine(vector<cv::Point3d> pts, cv::Point3d A, cv::Point3d B)
// input: line AB, collinear points
// output: whether AB is a good representation for points
// method: divide AB (or CD, which is endpoints of the projected points on AB)
// into n sub-segments, detect how many sub-segments containing
// at least one point(projected onto AB), if too few, then it implies invalid line
{
    int nCells = 10;//sysPara.num_cells_lineseg_range; // number of cells
    int* cells = new int[nCells];
    double ratio = 0.7;//sysPara.ratio_support_pts_on_line;
    for(int i=0; i<nCells; ++i) cells[i] = 0;
    int nPts = pts.size();
    // find 2 extremities of points along the line direction
    double minv=100, maxv=-100;
    int	   idx1 = 0, idx2 = 0;
    for(int i=0; i<nPts; ++i) {
        if ((pts[i]-A).dot(B-A) < minv) {
            minv = (pts[i]-A).dot(B-A);
            idx1 = i;
        }
        if ((pts[i]-A).dot(B-A) > maxv) {
            maxv = (pts[i]-A).dot(B-A);
            idx2 = i;
        }
    }
    cv::Point3d C = projectPt3d2Ln3d (pts[idx1], (A+B)*0.5, B-A);
    cv::Point3d D = projectPt3d2Ln3d (pts[idx2], (A+B)*0.5, B-A);
    double cd = cv::norm(D-C);
    if(cd < EPS) {
        delete[] cells;
        return false;
    }
    for(int i=0; i<nPts; ++i) {
        cv::Point3d X = pts[i];
        double lambda = abs((X-C).dot(D-C)/cd/cd); // 0 <= lambd <=1
        if (lambda>=1) {
            cells[nCells-1] += 1;
        } else {
            cells[(unsigned int)floor(lambda*10)] += 1;
        }
    }
    double sum = 0;
    for (int i=0; i<nCells; ++i) {
        //		cout<<cells[i]<<"\t";
        if (cells[i] > 0 )
            sum ++;
    }
    //	cout<<'\t'<<sum<<endl;
    delete[] cells;
    if(sum/nCells > ratio) {
        return true;
    } else {
        return false;
    }
}


void computeLine3d_svd (vector<cv::Point3d> pts, cv::Point3d& mean, cv::Point3d& drct)
// input: collinear 3d points with noise
// output: line direction vector and point
// method: linear equation, PCA
{
    int n = pts.size();
    mean = cv::Point3d(0,0,0);
    for(int i=0; i<n; ++i) {
        mean =  mean + pts[i];
    }
    mean = mean * (1.0/n);

    cv::Mat P(3,n,CV_64F);

    for(int i=0; i<n; ++i) {
        pts[i] =  pts[i] - mean;
        cvpt2mat(pts[i],0).copyTo(P.col(i));
    }
    cv::SVD svd(P.t());
    drct = mat2cvpt3d(svd.vt.row(0));
}

//使用TLS来求解直线方程

void computeLine3d_svd_tls(vector<cv::Point3d> pts, cv::Point3d & mean, cv:: Point3d& drct)
{
    int n = pts.size();
    cv::Mat X(3,2*n,CV_64F);
    //SVD分解


    cv::Mat P(3,n,CV_64F);

    for(int i=0; i<n; ++i) {
        pts[i] =  pts[i] - mean;
        cvpt2mat(pts[i],0).copyTo(P.col(i));
    }
    cv::SVD svd(P.t());
    drct = mat2cvpt3d(svd.vt.row(0));

}

void computeLine3d_svd (vector<RandomPoint3d> pts, cv::Point3d& mean, cv::Point3d& drct)
// input: collinear 3d points with noise
// output: line direction vector and point
// method: linear equation, PCA
{
    int n = pts.size();
    mean = cv::Point3d(0,0,0);
    for(int i=0; i<n; ++i) {
        mean =  mean + pts[i].pos;
    }
    mean = mean * (1.0/n);
    cv::Mat P(3,n,CV_64F);
    for(int i=0; i<n; ++i) {
        //	pts[i].pos =  pts[i].pos - mean;
        //	cvpt2mat(pts[i].pos,0).copyTo(P.col(i));
        double pos[3] = {pts[i].pos.x - mean.x,pts[i].pos.y - mean.y,pts[i].pos.z - mean.z};
        array2mat(pos,3).copyTo(P.col(i));
    }
    cv::SVD svd(P.t());
    drct = mat2cvpt3d(svd.vt.row(0));
}


cv::Point3d projectPt3d2Ln3d (const cv::Point3d& P, const cv::Point3d& mid, const cv::Point3d& drct)
// project a 3d point P to a 3d line (represented with midpt and direction)
{
    cv::Point3d A = mid;
    cv::Point3d B = mid + drct;
    cv::Point3d AB = B-A;
    cv::Point3d AP = P-A;
    return A + (AB.dot(AP)/(AB.dot(AB)))*AB;
}

cv::Mat array2mat(double a[], int n) // inhomo mat
// n is the size of a[]
{
    return cv::Mat(n,1,CV_64F,a);
}

cv::Point2d mat2cvpt (const cv::Mat& m)
// 3x1 mat => point
{
    if (m.cols * m.rows == 2)
        return cv::Point2d(m.at<double>(0), m.at<double>(1));
    if (m.cols * m.rows ==3)
        return cv::Point2d(m.at<double>(0)/m.at<double>(2),
                           m.at<double>(1)/m.at<double>(2));
    else
        cerr<<"input matrix dimmension wrong!";
}
cv::Point3d mat2cvpt3d (cv::Mat m)
// 3x1 mat => point
{
    if (m.cols * m.rows ==3)
        return cv::Point3d(m.at<double>(0),
                           m.at<double>(1),
                           m.at<double>(2));
    else
        cerr<<"input matrix dimmension wrong!";
}

cv::Mat cvpt2mat(const cv::Point3d& p, bool homo)
// this function is slow!
// return cv::Mat(3,1,CV_64,arrary) does not work!
{
    if (homo)
        return (cv::Mat_<double>(4,1)<<p.x, p.y, p.z, 1);
    else {
        return (cv::Mat_<double>(3,1)<<p.x, p.y, p.z);

    }
}

cv::Mat cvpt2mat(cv::Point2d p, bool homo)
{
    if (homo)
        return (cv::Mat_<double>(3,1)<<p.x, p.y, 1);
    else
        return (cv::Mat_<double>(2,1)<<p.x, p.y);
}

int computeMSLD (FrameLine& l, cv::Mat* xGradient, cv::Mat* yGradient)
// compute msld and gradient
{
    cv::Point2d gradient = l.getGradient(xGradient, yGradient);
    l.r = gradient;
    int s = 5 * xGradient->cols/800.0;
    double len = cv::norm(l.p-l.q);

    vector<vector<double> > GDM; //GDM.reserve(2*(int)len);
    double step = 1; // the step length between sample points on line segment
    for (int i=0; i*step < len; ++i) {
        vector<double> col; col.reserve(9);
        //	col.clear();
        cv::Point2d pt =    // compute point position on the line
                l.p + (l.q - l.p) * (i*step/len);
        bool fail = false;
        for (int j=-4; j <= 4; ++j ) { // 9 PSR for each point on line
            vector<double> psr(4);
            if (computeSubPSR (xGradient, yGradient, pt+j*s*gradient, s, gradient, psr)) {
                col.push_back(psr[0]);
                col.push_back(psr[1]);
                col.push_back(psr[2]);
                col.push_back(psr[3]);
            } else
                fail = true;
        }
        if (fail)
            continue;
        GDM.push_back(col);
    }

    cv::Mat MS(72, 1, CV_64F);
    if (GDM.size() ==0 ) {
        for (int i=0; i<MS.rows; ++i)
            MS.at<double>(i,0) = rand(); // if not computable, assign random num
        l.des = MS;
        return 0;
    }

    double gauss[9] = { 0.24142,0.30046,0.35127,0.38579,0.39804,
                        0.38579,0.35127,0.30046,0.24142};
    for (int i=0; i < 36; ++i) {
        double sum=0, sum2=0, mean, std;
        for (int j=0; j < GDM.size(); ++j) {
            GDM[j][i] = GDM[j][i] * gauss[i/4];
            sum += GDM[j][i];
            sum2 += GDM[j][i]*GDM[j][i];
        }
        mean = sum/GDM.size();
        std = sqrt(abs(sum2/GDM.size() - mean*mean));
        MS.at<double>(i,0)		= mean;
        MS.at<double>(i+36, 0)	= std;
    }
    // normalize mean and std vector, respcectively
    MS.rowRange(0,36) = MS.rowRange(0,36) / cv::norm(MS.rowRange(0,36));
    MS.rowRange(36,72) = MS.rowRange(36,72) / cv::norm(MS.rowRange(36,72));
    for (int i=0; i < MS.rows; ++i) {
        if (MS.at<double>(i,0) > 0.4)
            MS.at<double>(i,0) = 0.4;
    }
    MS = MS/cv::norm(MS);
    l.des.create(72, 1, CV_64F);
    l.des = MS;
    return 1;
}

int computeSubPSR (cv::Mat* xGradient, cv::Mat* yGradient,
                   cv::Point2d p, double s, cv::Point2d g, vector<double>& vs) {
    /* input: p - 2D point position
    s - side length of square region
    g - unit vector of gradient of line
    output: vs = (v1, v2, v3, v4)
    */
    double tl_x = floor(p.x - s/2), tl_y = floor(p.y - s/2);
    if (tl_x < 0 || tl_y < 0 ||
        tl_x+s+1 > xGradient->cols || tl_y+s+1 > xGradient->rows)
        return 0; // out of image
    double v1=0, v2=0, v3=0, v4=0;
    for (int x  = tl_x; x < tl_x+s; ++x) {
        for (int y = tl_y; y < tl_y+s; ++y) {
            //			cout<< xGradient->at<double>(y,x) <<","<<yGradient->at<double>(y,x) <<endl;
            //			cout<<"("<<y<<","<<x<<")"<<endl;
            double tmp1 =
                    xGradient->at<double>(y,x)*g.x + yGradient->at<double>(y,x)*g.y;
            double tmp2 =
                    xGradient->at<double>(y,x)*(-g.y) + yGradient->at<double>(y,x)*g.x;
            if ( tmp1 >= 0 )
                v1 = v1 + tmp1;
            else
                v2 = v2 - tmp1;
            if (tmp2 >= 0)
                v3 = v3 + tmp2;
            else
                v4 = v4 - tmp2;
        }
    }
    vs.resize(4);
    vs[0] = v1; vs[1] = v2;
    vs[2] = v3; vs[3] = v4;
    return 1;
}


FrameLines::FrameLines(cv::Point2d p_, cv::Point2d q_)
{
    p = p_;
    q = q_;
    l = cvpt2mat(p,1).cross(cvpt2mat(q,1));
    haveDepth = false;
    gid = -1;
}

cv::Point2d FrameLines::getGradient(cv::Mat* xGradient, cv::Mat* yGradient)
{
    cv::LineIterator iter(*xGradient, p, q, 8);
    double xSum=0, ySum=0;
    for (int i=0; i<iter.count; ++i, ++iter) {
        xSum += xGradient->at<double>(iter.pos());
        ySum += yGradient->at<double>(iter.pos());
    }
    double len = sqrt(xSum*xSum+ySum*ySum);
    return cv::Point2d(xSum/len, ySum/len);
}

double lineSegmentOverlap(const FrameLine& a, const FrameLine& b)
// compute the overlap length of two line segments in their parallel direction
{
    if(cv::norm(a.p-a.q) < cv::norm(b.p-b.q)) {// a is shorter than b
        double lambda_p = projectPt2d_to_line2d(a.p, b.p, b.q);
        double lambda_q = projectPt2d_to_line2d(a.q, b.p, b.q);
        if( (lambda_p < 0 && lambda_q < 0) || (lambda_p > 1 && lambda_q > 1) )
            return -1;
        else
            return abs(lambda_p - lambda_q) * cv::norm(b.p-b.q);
    } else {
        double lambda_p = projectPt2d_to_line2d(b.p, a.p, a.q);
        double lambda_q = projectPt2d_to_line2d(b.q, a.p, a.q);
        if( (lambda_p < 0 && lambda_q < 0) || (lambda_p > 1 && lambda_q > 1) )
            return -1;
        else
            return abs(lambda_p - lambda_q) * cv::norm(a.p-a.q);
    }
}

double line_to_line_dist2d(FrameLine& a, FrameLine& b)
// compute line to line distance by averaging 4 endpoint to line distances
// a and b must be almost parallel to make any sense
{
    return 0.25*pt_to_line_dist2d(a.p,b.lineEq2d)
           + 0.25*pt_to_line_dist2d(a.q, b.lineEq2d)
           + 0.25*pt_to_line_dist2d(b.p, a.lineEq2d)
           + 0.25*pt_to_line_dist2d(b.q, a.lineEq2d);
}

double projectPt2d_to_line2d(const cv::Point2d& X, const cv::Point2d& A, const cv::Point2d& B)
{
    // X' = lambda*A + (1-lambda)*B; X' is the projection of X on AB
    cv::Point2d BX = X-B, BA = A-B;
    double lambda = BX.dot(BA)/cv::norm(BA)/cv::norm(BA);
    return lambda;
}

double pt_to_line_dist2d(const cv::Point2d& p, double l[3])
// distance from point(x,y) to line ax+by+c=0;
// l=(a, b, c), p = (x,y)
{
    if(abs(l[0]*l[0]+l[1]*l[1]-1)>EPS) {
        cout<<"pt_to_line_dist2d: error,l should be normalized/initialized\n";
        exit(0);
    }
    double a = l[0],
            b = l[1],
            c = l[2],
            x = p.x,
            y = p.y;
    return abs((a*x+b*y+c))/sqrt(a*a+b*b);
}

void matchLine (vector<FrameLine> f1, vector<FrameLine> f2, vector<vector<int> >& matches)
// line segment matching (for loop closure)
// input:
// finishing in <10 ms
{

    double lineDistThresh  = 80; // pixel
    double lineAngleThresh = 25 * PI/180; // 30 degree
    double desDiffThresh   = 0.7;
    double lineOverlapThresh = -1; // pixels
    double ratio_dist_1st2nd = 0.7;

    cv::Mat desDiff = cv::Mat::zeros(f1.size(), f2.size(), CV_64F)+100;
#pragma omp  parallel for
    for(int i=0; i<f1.size(); ++i) {
        for(int j=0; j<f2.size(); ++j) {
            if((f1[i].r.dot(f2[j].r) > cos(lineAngleThresh)) && // angle between gradients
               (line_to_line_dist2d(f1[i],f2[j]) < lineDistThresh) &&
               (lineSegmentOverlap(f1[i],f2[j]) > lineOverlapThresh )) // line (parallel) distance
            {
                desDiff.at<double>(i,j) = cv::norm(f1[i].des - f2[j].des);
            }
        }
    }

    for(int i=0; i<desDiff.rows; ++i) {
        vector<int> onePairIdx;
        double minVal;
        cv::Point minPos;
        cv::minMaxLoc(desDiff.row(i),&minVal,NULL,&minPos,NULL);
        if (minVal < desDiffThresh) {
            double minV;
            cv::Point minP;
            cv::minMaxLoc(desDiff.col(minPos.x),&minV,NULL,&minP,NULL);
            if (i==minP.y) {    // commnent this for more potential matches
                //further check distance ratio
                double rowmin2 = 100, colmin2 = 100;
                for(int j=0; j<desDiff.cols; ++j) {
                    if (j == minPos.x) continue;
                    if (rowmin2 > desDiff.at<double>(i,j)) {
                        rowmin2 = desDiff.at<double>(i,j);
                    }
                }
                for(int j=0; j<desDiff.rows; ++j) {
                    if (j == minP.y) continue;
                    if (colmin2 > desDiff.at<double>(j,minPos.x)) {
                        colmin2 = desDiff.at<double>(j,minPos.x);
                    }
                }
                if(rowmin2*ratio_dist_1st2nd > minVal && colmin2*ratio_dist_1st2nd > minVal) {
                    onePairIdx.push_back(i);
                    onePairIdx.push_back(minPos.x);
                    matches.push_back(onePairIdx);
                }
            }
        }
    }
}

void trackLine (vector<FrameLine> f1, vector<FrameLine> f2, vector<vector<int> >& matches)
// line segment tracking
// input:
// finishing in <10 ms
{

    double lineDistThresh  = 25; // pixel
    double lineAngleThresh = 25 * PI/180; // 30 degree
    double desDiffThresh   = 0.85;
    double lineOverlapThresh = 3; // pixels
    double ratio_dist_1st2nd = 0.7;

    //sysPara.fast_motion==1

    lineDistThresh = 45;
    lineAngleThresh = 30 * PI/180;
    desDiffThresh   = 0.85;
    lineOverlapThresh = -1;


    //dark_light ==false
//    if(sysPara.dark_ligthing) {
//        ratio_dist_1st2nd = 0.85;
//        desDiffThresh = 1.5;
//        lineDistThresh = 20;
//        lineAngleThresh = 10 * PI/180;
//
//    }

    if(f1.size()==0 || f2.size()==0) {
        return;
    }

    cv::Mat desDiff = cv::Mat::zeros(f1.size(), f2.size(), CV_64F)+100;
#pragma omp  parallel for
    for(int i=0; i<f1.size(); ++i) {
        for(int j=0; j<f2.size(); ++j) {
            //r is image line gradient.
            if((f1[i].r.dot(f2[j].r) > cos(lineAngleThresh)) && // angle between gradients
               (line_to_line_dist2d(f1[i],f2[j]) < lineDistThresh) &&
               (lineSegmentOverlap(f1[i],f2[j]) > lineOverlapThresh )) // line (parallel) distance
            {
                //cout<<"f1[i].r.dot(f2[j].r："<<f1[i].r.dot(f2[j].r)<<endl;
                desDiff.at<double>(i,j) = cv::norm(f1[i].des - f2[j].des);
            }
        }
    }

    for(int i=0; i<desDiff.rows; ++i) {
        vector<int> onePairIdx;
        double minVal;
        cv::Point minPos;
        cv::minMaxLoc(desDiff.row(i),&minVal,NULL,&minPos,NULL);
        if (minVal < desDiffThresh) {
            double minV;
            cv::Point minP;
            cv::minMaxLoc(desDiff.col(minPos.x),&minV,NULL,&minP,NULL);
            if (i==minP.y) {    // commnent this for more potential matches
                //further check distance ratio
                double rowmin2 = 100, colmin2 = 100;
                for(int j=0; j<desDiff.cols; ++j) {
                    if (j == minPos.x) continue;
                    if (rowmin2 > desDiff.at<double>(i,j)) {
                        rowmin2 = desDiff.at<double>(i,j);
                    }
                }
                for(int j=0; j<desDiff.rows; ++j) {
                    if (j == minP.y) continue;
                    if (colmin2 > desDiff.at<double>(j,minPos.x)) {
                        colmin2 = desDiff.at<double>(j,minPos.x);
                    }
                }
                if(rowmin2*ratio_dist_1st2nd > minVal && colmin2*ratio_dist_1st2nd > minVal) {
                    onePairIdx.push_back(i);
                    onePairIdx.push_back(minPos.x);
                    matches.push_back(onePairIdx);
                }
            }
        }
    }
}

bool get_pt_3d (cv::Point2d p2, cv::Point3d& p3, const cv::Mat& depth)
{
    if(p2.x<0 || p2.y<0 || p2.x >= depth.cols || p2.y >= depth.rows )
        return false;
    int row, col; // nearest pixel for pt
    if((floor(p2.x) == p2.x) && (floor(p2.y) == p2.y)) {// boundary issue
        col = max(int(p2.x-1),0);
        row = max(int(p2.y-1),0);
    } else {
        col = int(p2.x);
        row = int(p2.y);
    }

    if(depth.at<double>(row,col) < EPS) { // no depth info
        return false;
    } else {
        double zval = depth.at<double>(row,col)/5000; // in meter, z-value
        //cv::Point2d tmp = mat2cvpt(K.inv()*cvpt2mat(p2))*zval;
        //p3.x = tmp.x;
        //p3.y = tmp.y;
        //p3.z = zval;
        return true;
    }
}

void MLEstimateLine3d_compact (RandomLine3d& line,	int maxIter)
// optimally estimate a 3d line from a set of collinear random 3d points
// 3d line is represented by two points
{
//	static double acum=0;
//	static int count = 0;
//	MyTimer timer; 	timer.start();

    // ----- preprocessing: find 2 extremities of points along the line direction -----
    double minv=100, maxv=-100;
    int	   idx_end1 = 0, idx_end2 = 0;
    for(int i=0; i<line.pts.size(); ++i) {
        double dproduct = (line.pts[i].pos-line.A).dot(line.A-line.B);
        if ( dproduct < minv) {
            minv = dproduct;
            idx_end1 = i;
        }
        if (dproduct > maxv) {
            maxv = dproduct;
            idx_end2 = i;
        }
    }
    if(idx_end1 > idx_end2) swap(idx_end1, idx_end2); // ensure idx_end1 < idx_end2
    // ----- LM parameter setting -----
    double opts[5],info[10];//LM_OPTS_SZ], info[LM_INFO_SZ];
    opts[0] = 1E-03;//LM_INIT_MU; //
    opts[1] = 1E-10; // gradient threshold, original 1e-15
    opts[2] = 1E-20; // relative para change threshold? original 1e-50
    opts[3] = 1E-20; // error threshold (below it, stop)
    opts[4] = 1E-03;//LM_DIFF_DELTA;

    // ----- optimization parameters -----
    Data_MLEstimateLine3d_Compact data(line.pts);
    data.idx1 = idx_end1;
    data.idx2 = idx_end2;
    vector<double> paraVec, measVec;
    for(int i = 0; i < line.pts.size(); ++i) {
        measVec.push_back(0);
        if (i == idx_end1 || i == idx_end2) {
            paraVec.push_back(line.pts[i].pos.x);
            paraVec.push_back(line.pts[i].pos.y);
            paraVec.push_back(line.pts[i].pos.z);
        }
    }
    int numPara = paraVec.size();
    double* para = new double[numPara];
    for (int i=0; i<numPara; ++i) {
        para[i] = paraVec[i];
    }
    int numMeas = measVec.size();
    double* meas = new double[numMeas];
    for ( int i=0; i<numMeas; ++i) {
        meas[i] = measVec[i];
    }
    // ----- start LM solver -----
    //int ret = dlevmar_dif(costFun_MLEstimateLine3d_compact, para, meas, numPara, numMeas,
    //                      maxIter, opts, info, NULL, NULL, (void*)&data);
//	termReason((int)info[6]);
//	cout<<endl<<endl;

    // ----- compute cov of MLE result -----
    cv::Mat cov_meas_inv = cv::Mat::zeros(3*line.pts.size(), 3*line.pts.size(), CV_64F);
    cv::Mat tmp;
    double* p = new double[line.pts.size()+4]; // mimic the full/long parameter vector
    int idx = 0;
    for(int i=0; i<line.pts.size();++i) {
        tmp = line.pts[i].cov.inv();
        tmp.copyTo(cov_meas_inv.rowRange(i*3,i*3+3).colRange(i*3,i*3+3));
        if(i==idx_end1) {
            p[idx] = para[0];
            p[idx+1] = para[1];
            p[idx+2] = para[2];
            idx = idx + 3;
        } else if(i==idx_end2){
            p[idx] = para[3];
            p[idx+1] = para[4];
            p[idx+2] = para[5];
            idx = idx + 3;
        }else {
            // project pt to line to get ratio
            p[idx] = 1- closest_3dpt_ratio_online_mah (line.pts[i],
                                                       cv::Point3d(para[0],para[1],para[2]), cv::Point3d(para[3],para[4],para[5]));
            ++idx;
        }
    }

    // ---- compute line endpoint uncerainty ----
    // MlestimateLine3dCov (p, line.pts.size()+4, idx_end1, idx_end2, cov_meas_inv, line.covA, line.covB);

    // refine line endpoint positions
    line.A = cv::Point3d (p[idx_end1],p[idx_end1+1],p[idx_end1+2]);
    line.B = cv::Point3d (p[idx_end2+2],p[idx_end2+3],p[idx_end2+4]);
    line.rndA = RandomPoint3d(line.A, line.covA);
    line.rndB = RandomPoint3d(line.B, line.covB);
#ifdef SAVE_MEMORY
    line.pts.clear();
#endif
//	timer.end();
//	count +=1;
//	acum += timer.time_ms;
//	if(count%50==0)	cout<<"mle-cmp accumulated: "<<acum<<endl;
    delete[] meas;
    delete[] para;
    delete[] p;
}
vector<int> computeRelativeMotion_Ransac (vector<RandomLine3d> a, vector<RandomLine3d> b, cv::Mat& Ro, cv::Mat& to)
// compute relative pose between two cameras using 3d line correspondences
// ransac
{
    if (a.size()<3) {
        return vector<int>();
    }
    //	MyTimer t1; t1.start();
    // convert to the representation of Zhang's paper
    vector<vector<double> > aA(a.size()), aB(a.size()), aAB(a.size()), bAB(a.size());
    for(int i=0; i<a.size(); ++i) {
        cv::Point3d l = a[i].B - a[i].A;
        cv::Point3d m = (a[i].A + a[i].B) * 0.5;
        a[i].u = l * (1/cv::norm(l));
        a[i].d = a[i].u.cross(m);
        l = b[i].B - b[i].A;
        m = (b[i].A + b[i].B) * 0.5;
        b[i].u = l * (1/cv::norm(l));
        b[i].d = b[i].u.cross(m);
        aA[i].resize(3);
        aB[i].resize(3);
        aAB[i].resize(3);
        bAB[i].resize(3);
        aA[i][0] = a[i].A.x;
        aA[i][1] = a[i].A.y;
        aA[i][2] = a[i].A.z;
        aB[i][0] = a[i].B.x;
        aB[i][1] = a[i].B.y;
        aB[i][2] = a[i].B.z;
        aAB[i][0] = a[i].A.x-a[i].B.x;
        aAB[i][1] = a[i].A.y-a[i].B.y;
        aAB[i][2] = a[i].A.z-a[i].B.z;
        bAB[i][0] = b[i].A.x-b[i].B.x;
        bAB[i][1] = b[i].A.y-b[i].B.y;
        bAB[i][2] = b[i].A.z-b[i].B.z;
    }
    //cout<<"start RANSAC"<<endl;
    // ----- start ransac -----
    int minSolSetSize = 3, maxIters = 500;
    double distThresh =0.05;// sysPara.pt2line3d_dist_relmotion; // in meter
    double angThresh  = 10;//sysPara.line3d_angle_relmotion; // deg
    double lineAngleThresh_degeneracy = 5*PI/180; //5 degree

    vector<int> indexes;
    for(int i=0; i<a.size(); ++i)	indexes.push_back(i);
    int iter = 0;
    vector<int> maxConSet;
    cv::Mat bR, bt;
    while(iter<maxIters) {
        vector<int> inlier;
        iter++;
        random_unique(indexes.begin(), indexes.end(),minSolSetSize);// shuffle

        vector<RandomLine3d> suba, subb;
        for(int i=0; i<minSolSetSize; ++i) {
            suba.push_back(a[indexes[i]]);
            subb.push_back(b[indexes[i]]);
        }
        // ---- check degeneracy ----
        bool degenerate = true;
        // if at least one pair is not parallel, then it's non-degenerate
        for(int i=0; i<minSolSetSize; ++i){
            for(int j=i+1; j<minSolSetSize; ++j) {
                if(abs(suba[i].u.dot(suba[j].u)) < cos(lineAngleThresh_degeneracy)) {
                    degenerate = false;
                    break;
                }
            }
            if(!degenerate)
                break;
        }
        if(degenerate) continue; // degenerate set is not usable
        // cout<<"start svd FOR MOTION"<<endl;
        cv::Mat R, t;
        computeRelativeMotion_svd(suba, subb, R, t);
        // find consensus
        for(int i=0; i<a.size(); ++i) {
            /*		double aiA[3] = {a[i].A.x,a[i].A.y,a[i].A.z},
                    aiB[3] = {a[i].B.x,a[i].B.y,a[i].B.z},
                    aiA_B[3] = {(a[i].A-a[i].B).x,(a[i].A-a[i].B).y,(a[i].A-a[i].B).z},
                    biA_B[3] = {(b[i].A-b[i].B).x,(b[i].A-b[i].B).y,(b[i].A-b[i].B).z};
                    double dist = 0.5*dist3d_pt_line (mat2cvpt3d(R*array2mat(aiA,3)+t), b[i].A, b[i].B)
                                + 0.5*dist3d_pt_line (mat2cvpt3d(R*array2mat(aiB,3)+t), b[i].A, b[i].B);
                    double angle = 180*acos(abs((R*array2mat(aiA_B, 3)).dot(array2mat(biA_B,3))/
                                    cv::norm(a[i].A - a[i].B)/cv::norm(b[i].A - b[i].B)))/PI; // degree
            */
            double dist = 0.5*dist3d_pt_line (mat2cvpt3d(R*array2mat(&aA[i][0],3)+t), b[i].A, b[i].B)
                          + 0.5*dist3d_pt_line (mat2cvpt3d(R*array2mat(&aB[i][0],3)+t), b[i].A, b[i].B);
            double angle = 180*acos(abs((R*array2mat(&aAB[i][0], 3)).dot(array2mat(&bAB[i][0],3))/
                                        cv::norm(a[i].A - a[i].B)/cv::norm(b[i].A - b[i].B)))/PI;
            if(dist < distThresh && angle < angThresh) {
                inlier.push_back(i);
            }
        }
        if(inlier.size() > maxConSet.size()) {
            maxConSet = inlier;
            bR = R;
            bt = t;
        }

    }
    if(maxConSet.size()<1)
        return maxConSet;
    Ro = bR; to = bt;
    if(maxConSet.size()<4)
        return maxConSet;
    // ---- apply svd to all inliers ----
    vector<RandomLine3d> ina, inb;
    for(int i=0; i<maxConSet.size();++i) {
        ina.push_back(a[maxConSet[i]]);
        inb.push_back(b[maxConSet[i]]);
    }

    computeRelativeMotion_svd(ina, inb, Ro, to);
    //cout<<"Ro"<<Ro<<endl;
    //optimizeRelmotion(ina, inb, Ro, to);
    cv::Mat R = Ro, t = to;
    vector<int> prevConSet;
    while(1) {
        vector<int> conset;
        for(int i=0; i<a.size(); ++i) {
            /*	double dist = 0.5*dist3d_pt_line (mat2cvpt3d(R*cvpt2mat(a[i].A,0)+t), b[i].A, b[i].B)
                    + 0.5*dist3d_pt_line (mat2cvpt3d(R*cvpt2mat(a[i].B,0)+t), b[i].A, b[i].B);
                double angle = 180*acos(abs((R*cvpt2mat(a[i].A - a[i].B, 0)).dot(cvpt2mat(b[i].A-b[i].B,0))/
                    cv::norm(a[i].A - a[i].B)/cv::norm(b[i].A - b[i].B)))/PI; // degree
                double aiA[3] = {a[i].A.x,a[i].A.y,a[i].A.z},
                aiB[3] = {a[i].B.x,a[i].B.y,a[i].B.z},
                aiA_B[3] = {(a[i].A-a[i].B).x,(a[i].A-a[i].B).y,(a[i].A-a[i].B).z},
                biA_B[3] = {(b[i].A-b[i].B).x,(b[i].A-b[i].B).y,(b[i].A-b[i].B).z};
            */	double dist = 0.5*dist3d_pt_line (mat2cvpt3d(R*array2mat(&aA[i][0],3)+t), b[i].A, b[i].B)
                                + 0.5*dist3d_pt_line (mat2cvpt3d(R*array2mat(&aB[i][0],3)+t), b[i].A, b[i].B);
            double angle = 180*acos(abs((R*array2mat(&aAB[i][0], 3)).dot(array2mat(&bAB[i][0],3))/
                                        cv::norm(a[i].A - a[i].B)/cv::norm(b[i].A - b[i].B)))/PI;
            if(dist < distThresh && angle < angThresh) {
                conset.push_back(i);
            }
        }
        if(conset.size() <= prevConSet.size())
            break;
        else {
            prevConSet = conset;
            Ro = R;
            to = t;
            //		cout<<Ro<<endl<<to<<'\t'<<prevConSet.size()<<endl<<endl;
            ina.clear(); inb.clear();
            for(int i=0; i<prevConSet.size();++i) {
                ina.push_back(a[prevConSet[i]]);
                inb.push_back(b[prevConSet[i]]);
            }
            //optimizeRelmotion(ina, inb, R, t);
        }
    }
    return prevConSet;
}

void costFun_MLEstimateLine3d_compact(double *p, double *error, int m, int n, void *adata)
{
    struct Data_MLEstimateLine3d_Compact* dptr;
    dptr = (struct Data_MLEstimateLine3d_Compact *) adata;
    int curParaIdx = 0, curErrIdx = 0;
    int idx1 = dptr->idx1, idx2 = dptr->idx2;
    cv::Mat a(3,1,CV_64F, p), b(3,1,CV_64F, &p[3]);
    cv::Point3d ap(p[0], p[1], p[2]);
    cv::Point3d bp(p[3], p[4], p[5]);
    for(int i=0; i<dptr->pts.size(); ++i) {
        if(i==idx1) {
            cv::Mat mdist = (a-cvpt2mat(dptr->pts[i].pos,0)).t() *dptr->pts[i].cov.inv()*(a-cvpt2mat(dptr->pts[i].pos,0));
            error[i] = mdist.at<double>(0);
        } else if(i==idx2) {
            cv::Mat mdist = (b-cvpt2mat(dptr->pts[i].pos,0)).t() *dptr->pts[i].cov.inv()*(b-cvpt2mat(dptr->pts[i].pos,0));
            error[i] = mdist.at<double>(0);
        } else {
            //error[i] = mah_dist3d_pt_line (dptr->pts[i], ap, bp);
        }
    }
    double cost=0;
    static int c=0;
    c++;
    assert(dptr->pts.size()==n);
    for(int i=0; i<dptr->pts.size(); ++i)
        cost+=error[i]*error[i];
//	cout<<cost<<"\t";
}


double closest_3dpt_ratio_online_mah (const RandomPoint3d& pt, cv::Point3d q1, cv::Point3d q2)
// compute the closest point using the Mahalanobis distance from a random 3d point p to line (q1,q2)
// return the ratio t, such that q1 + t * (q2 - q1) is the closest point
{
    if (pt.U.cols != 3)	{
        cerr<<"Error in mah_dist3d_pt_line: R matrix must be 3x3"<<endl;
        exit(0);
    }

    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    r11 = pt.U.at<double>(0,0);
    r12 = pt.U.at<double>(0,1);
    r13 = pt.U.at<double>(0,2);
    r21 = pt.U.at<double>(1,0);
    r22 = pt.U.at<double>(1,1);
    r23 = pt.U.at<double>(1,2);
    r31 = pt.U.at<double>(2,0);
    r32 = pt.U.at<double>(2,1);
    r33 = pt.U.at<double>(2,2);
    cv::Point3d q1_p = q1 - pt.pos, q2_p = q2 - pt.pos;

    double s0 = sqrt(pt.W.at<double>(0)), s1 = sqrt(pt.W.at<double>(1)), s2 = sqrt(pt.W.at<double>(2));
    cv::Point3d q1n((q1_p.x * r11 + q1_p.y * r21 + q1_p.z * r31)/s0,
                    (q1_p.x * r12 + q1_p.y * r22 + q1_p.z * r32)/s1,
                    (q1_p.x * r13 + q1_p.y * r23 + q1_p.z * r33)/s2),
            q2n((q2_p.x * r11 + q2_p.y * r21 + q2_p.z * r31)/s0,
                (q2_p.x * r12 + q2_p.y * r22 + q2_p.z * r32)/s1,
                (q2_p.x * r13 + q2_p.y * r23 + q2_p.z * r33)/s2);
    double t = - q1n.dot(q2n-q1n)/((q2n-q1n).dot(q2n-q1n));
    return t;
}

bool computeRelativeMotion_svd (vector<cv::Point3d> a, vector<cv::Point3d> b, cv::Mat& R, cv::Mat& t)
{
    int n = a.size();
    if (n<3) {
        return false;
    }
    // compute centroid
    cv::Point3d ac(0,0,0), bc(0,0,0);
    for (int i=0; i<n; ++i) {
        ac += a[i];
        bc += b[i];
    }
    ac = ac * (1.0/n);
    bc = bc * (1.0/n);

    cv::Mat H = cv::Mat::zeros(3,3,CV_64F);
    for(int i=0; i<n; ++i) {
        H = H + cvpt2mat(a[i]-ac, 0) * cvpt2mat(b[i]-bc,0).t();
    }
    cv::SVD svd(H);
    R = (svd.vt * svd.u).t();
    if(cv::determinant(R)<0)
        R.col(2) = R.col(2) * -1;

    t = -R*cvpt2mat(ac,0) + cvpt2mat(bc,0);
    return true;
}

void computeRelativeMotion_svd (vector<RandomLine3d> a, vector<RandomLine3d> b, cv::Mat& R, cv::Mat& t)
// input needs at least 2 correspondences of non-parallel lines
// the resulting R and t works as below: x'=Rx+t for point pair(x,x');
{
    if(a.size()<2)	{
        cerr<<"Error in computeRelativeMotion_svd: input needs at least 2 pairs!\n";
        return;
    }
    cv::Mat A = cv::Mat::zeros(4,4,CV_64F);
    for(int i=0; i<a.size(); ++i) {
        cv::Mat Ai = cv::Mat::zeros(4,4,CV_64F);
        Ai.at<double>(0,1) = (a[i].u-b[i].u).x;
        Ai.at<double>(0,2) = (a[i].u-b[i].u).y;
        Ai.at<double>(0,3) = (a[i].u-b[i].u).z;
        Ai.at<double>(1,0) = (b[i].u-a[i].u).x;
        Ai.at<double>(2,0) = (b[i].u-a[i].u).y;
        Ai.at<double>(3,0) = (b[i].u-a[i].u).z;
        vec2SkewMat(a[i].u+b[i].u).copyTo(Ai.rowRange(1,4).colRange(1,4));
        A = A + Ai.t()*Ai;
    }
    cv::SVD svd(A);
    cv::Mat q = svd.u.col(3);
    //cout<<"q="<<q<<endl;
    R = q2r(q);
    cv::Mat uu = cv::Mat::zeros(3,3,CV_64F),
            udr= cv::Mat::zeros(3,1,CV_64F);
    for(int i=0; i<a.size(); ++i) {
        uu = uu + vec2SkewMat(b[i].u)*vec2SkewMat(b[i].u).t();
        udr = udr + vec2SkewMat(b[i].u).t()* (cvpt2mat(b[i].d,0)-R*cvpt2mat(a[i].d,0));
    }
    t = uu.inv()*udr;
}


cv::Mat vec2SkewMat (cv::Mat vec)
{
    cv::Mat m = (cv::Mat_<double>(3,3) <<
                                       0, -vec.at<double>(2), vec.at<double>(1),
            vec.at<double>(2), 0, -vec.at<double>(0),
            -vec.at<double>(1), vec.at<double>(0), 0);
    return m;
}
cv::Mat vec2SkewMat (cv::Point3d vec)
{
    cv::Mat m = (cv::Mat_<double>(3,3) <<
                                       0, -vec.z, vec.y,
            vec.z, 0, -vec.x,
            -vec.y, vec.x, 0);
    return m;
}



void optimizeRelmotion(vector<RandomLine3d> a, vector<RandomLine3d> b, cv::Mat& R, cv::Mat& t)
{

    cv::Mat q = r2q(R);
    // ----- LM parameter setting -----
    double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
    opts[0] = LM_INIT_MU; //
    opts[1] = 1E-10; // gradient threshold, original 1e-15
    opts[2] = 1E-20; // relative para change threshold? original 1e-50
    opts[3] = 1E-20; // error threshold (below it, stop)
    opts[4] = LM_DIFF_DELTA;
    int maxIter = 500;

    // ----- optimization parameters -----
    int numPara = 7;
    double* para = new double[numPara];
    para[0] = q.at<double>(0);
    para[1] = q.at<double>(1);
    para[2] = q.at<double>(2);
    para[3] = q.at<double>(3);
    para[4] = t.at<double>(0);
    para[5] = t.at<double>(1);
    para[6] = t.at<double>(2);

    // ----- measurements -----
    int numMeas = a.size();
    double* meas = new double[numMeas];
    for(int i=0; i<numMeas; ++i) meas[i] = 0;

    Data_optimizeRelmotion data(a,b);
    // ----- start LM solver -----
    MyTimer timer; 	timer.start();
    //  int ret = dlevmar_dif(costFun_optimizeRelmotion, para, meas, numPara, numMeas,
    //                 maxIter, opts, info, NULL, NULL, (void*)&data);
    //	timer.end();	cout<<"optimizeRelmotion Time used: "<<timer.time_ms<<" ms. "<<endl;


    R = q2r(para);
    t = (cv::Mat_<double>(3,1)<<para[4],para[5],para[6]);
    delete[] meas;
    delete[] para;

}

cv::Mat r2q(cv::Mat R)
{
    double t = R.at<double>(0,0)+R.at<double>(1,1)+R.at<double>(2,2);
    double r = sqrt(1+t);
    double s = 0.5/r;
    double w = 0.5*r;
    double x = (R.at<double>(2,1)-R.at<double>(1,2))*s;
    double y = (R.at<double>(0,2)-R.at<double>(2,0))*s;
    double z = (R.at<double>(1,0)-R.at<double>(0,1))*s;
    cv::Mat q = (cv::Mat_<double>(4,1)<<w,x,y,z);
    return q;
}
cv::Mat q2r (cv::Mat q)
// input: unit quaternion representing rotation
// output: 3x3 rotation matrix
// note: q=(a,b,c,d)=a + b i + c j + d k, where (b,c,d) is the rotation axis
{
    double a = q.at<double>(0),	b = q.at<double>(1),
            c = q.at<double>(2), d = q.at<double>(3);
    double nm = sqrt(a*a+b*b+c*c+d*d);
    a = a/nm;
    b = b/nm;
    c = c/nm;
    d = d/nm;
    cv::Mat R = (cv::Mat_<double>(3,3)<<
                                      a*a+b*b-c*c-d*d,	2*b*c-2*a*d,		2*b*d+2*a*c,
            2*b*c+2*a*d,		a*a-b*b+c*c-d*d,	2*c*d-2*a*b,
            2*b*d-2*a*c,		2*c*d+2*a*b,		a*a-b*b-c*c+d*d);
    return R.clone();
}
void computeLine3d_svd (const vector<RandomPoint3d>& pts, const vector<int>& idx, cv::Point3d& mean, cv::Point3d& drct)
// input: collinear 3d points with noise
// output: line direction vector and point
// method: linear equation, PCA
{
    int n = idx.size();
    mean = cv::Point3d(0,0,0);
    for(int i=0; i<n; ++i) {
        mean =  mean + pts[idx[i]].pos;
    }
    mean = mean * (1.0/n);
    cv::Mat P(3,n,CV_64F);
    for(int i=0; i<n; ++i) {
        //	pts[i].pos =  pts[i].pos - mean;
        //	cvpt2mat(pts[i].pos,0).copyTo(P.col(i));
        double pos[3] = {pts[idx[i]].pos.x-mean.x, pts[idx[i]].pos.y-mean.y, pts[idx[i]].pos.z-mean.z};
        array2mat(pos,3).copyTo(P.col(i));
    }

    cv::SVD svd(P.t(), cv::SVD::MODIFY_A);  // FULL_UV is 60 times slower

    drct = mat2cvpt3d(svd.vt.row(0));
}


double depthStdDev (double d)
// standard deviation of depth d
// in meter
{
    double c1, c2, c3;

    c1 =  0.00273;//depth_stdev_coeff_c1;
    c2 = 0.00074;//depth_stdev_coeff_c2;
    c3 = -0.00058;//depth_stdev_coeff_c3;

    return c1*d*d + c2*d + c3;

//	return sysPara.depth_stdev_coeff_c1*d*d + sysPara.depth_stdev_coeff_c2*d + sysPara.depth_stdev_coeff_c3;
}
RandomPoint3d compPt3dCov (cv::Point3d pt, cv::Mat K, double time_diff_sec)
{
    RandomPoint3d rp;

    //cout<<" Lineextractor: Pt"<<pt.x<<","<<pt.y<<","<<pt.z<<K.at<double>(0,0)<<","<<K.at<double>(0,1)<<endl;

    double f = K.at<double>(0,0), // focal length
            cu = K.at<double>(0,2),
            cv = K.at<double>(1,2);
    double sigma_impt = 1;//stdev_sample_pt_imgline; // std dev of image sample point

	//// opencv mat operation is slower than armadillo
	cv::Mat J0 = (cv::Mat_<double>(3,3)<< pt.z/f, 0, pt.x/pt.z,
		0, pt.z/f, pt.y/pt.z,
		0,0,1);

	cv::Mat cov_g_d0 = (cv::Mat_<double>(3,3)<<1, 0, 0,
		0, 1, 0,
		0, 0, depthStdDev(pt.z)*depthStdDev(pt.z));
	cv::Mat cov0 = J0 * cov_g_d0 * J0.t();
    //cout<<"Lineextractor: depthSt"<<depthStdDev(pt.z)*depthStdDev(pt.z)<<endl;
    rp.cov=cov0;
    rp.pos=pt;


    rp.xyz[0] = pt.x;
    rp.xyz[1] = pt.y;
    rp.xyz[2] = pt.z;

    cv::SVD svd(cov0);
    rp.U = svd.u.clone();
    rp.W = svd.w.clone();
    //cout<<" Lineextractor: svd.w"<<svd.w.at<double>(0)<<","<< svd.w.at<double>(1)<<","<<svd.w.at<double>(2)<<endl;
    rp.W_sqrt[0] = sqrt(svd.w.at<double>(0));
    rp.W_sqrt[1] = sqrt(svd.w.at<double>(1));
    rp.W_sqrt[2] = sqrt(svd.w.at<double>(2));

    cv::Mat D = (cv::Mat_<double>(3,3)<<1/rp.W_sqrt[0], 0, 0,
            0, 1/rp.W_sqrt[1], 0,
            0, 0, 1/rp.W_sqrt[2]);
    //cout<<" Lineextractor: w_sqrt"<<rp.W_sqrt[0]<<","<< rp.W_sqrt[1]<<","<< rp.W_sqrt[2]<<endl;
    cv::Mat du = D*rp.U.t();
    //cout<<" Lineextractor: du"<<du.at<double>(0,0)<<","<<du.at<double>(0,1)<<","<<du.at<double>(0,2)<<endl;
    rp.DU[0] = du.at<double>(0,0);  rp.DU[1] = du.at<double>(0,1);  rp.DU[2] = du.at<double>(0,2);
    rp.DU[3] = du.at<double>(1,0);  rp.DU[4] = du.at<double>(1,1);  rp.DU[5] = du.at<double>(1,2);
    rp.DU[6] = du.at<double>(2,0);  rp.DU[7] = du.at<double>(2,1);  rp.DU[8] = du.at<double>(2,2);
    rp.dux[0] =  rp.DU[0]* rp.pos.x +  rp.DU[1]* rp.pos.y +  rp.DU[2]* rp.pos.z;
    rp.dux[1] =  rp.DU[3]* rp.pos.x +  rp.DU[4]* rp.pos.y +  rp.DU[5]* rp.pos.z;
    rp.dux[2] =  rp.DU[6]* rp.pos.x +  rp.DU[7]* rp.pos.y +  rp.DU[8]* rp.pos.z;



	return rp;

/*
    arma::mat J(3,3,arma::fill::zeros), cov_g_d(3,3,arma::fill::zeros);
    J(0,0) = pt.z/f; J(0,2) = pt.x/pt.z;
    J(1,1) = pt.z/f; J(1,2) = pt.y/pt.z; J(2,2) = 1;
    cov_g_d(0,0) = sigma_impt*sigma_impt;
    cov_g_d(1,1) = sigma_impt*sigma_impt;
    cov_g_d(2,2) = depthStdDev(pt.z, time_diff_sec)*depthStdDev(pt.z, time_diff_sec);
    arma::mat cov = J*cov_g_d*J.t();

    cv::Mat c = (cv::Mat_<double>(3,3) << cov(0,0), cov(0,1), cov(0,2),
            cov(1,0), cov(1,1), cov(1,2),
            cov(2,0), cov(2,1), cov(2,2));
    return RandomPoint3d(pt,c);
*/
}
RandomLine3d extract3dline_mahdist(const vector<RandomPoint3d>& pts)
// extract a single 3d line from point clouds using ransac and mahalanobis distance
// input: 3d points and covariances
// output: inlier points, line parameters: midpt and direction
{
    //cout<<"Lineextractor: extract 3d line"<<pts.size()<<endl;
    int maxIterNo = min(10, int(pts.size()*(pts.size()-1)*0.5));
    double distThresh = 1.5 ;;//pt2line_mahdist_extractline; // meter
    // distance threshold should be adapted to line length and depth
    int minSolSetSize = 2;

    vector<int> indexes(pts.size());
    for (int i=0; i<indexes.size(); ++i) indexes[i]=i;
    vector<int> maxInlierSet;
    RandomPoint3d bestA, bestB;
    for(int iter=0; iter<maxIterNo;iter++) {
        vector<int> inlierSet;
        random_unique(indexes.begin(), indexes.end(),minSolSetSize);// shuffle
        const RandomPoint3d& A = pts[indexes[0]];

        const RandomPoint3d& B = pts[indexes[1]];
        //cout<<"A:"<<A.xyz[0]<<","<<A.xyz[1]<<","<<A.xyz[2]<<".B:"<<B.xyz[2]<<endl;

        if (cv::norm(B.pos-A.pos) < EPS ) continue;
        for (int i=0; i<pts.size(); ++i) {
            // compute distance to AB
            double dist = mah_dist3d_pt_line(pts[i], A.pos, B.pos);
            //cout<<"dist takes "<<dist<<endl;
            //cout<<"dist:A pos"<<A.pos.x<<","<<A.xyz[2]<<endl;
            //cout<<"Lineextractor: dist"<<dist<<endl;
            if (dist<distThresh) {
                inlierSet.push_back(i);
            }
        }
        if(inlierSet.size() > maxInlierSet.size())	{
            vector<RandomPoint3d> inlierPts(inlierSet.size());
            for(int ii=0; ii<inlierSet.size(); ++ii)
                inlierPts[ii]=pts[inlierSet[ii]];
            if (verify3dLine(inlierPts, A.pos, B.pos)) {
                maxInlierSet = inlierSet;
                bestA = pts[indexes[0]]; bestB = pts[indexes[1]];
            }
        }
        	//cout<<"Lineextractor:"<<iter<<'\t'<<maxInlierSet.size()<<endl;
        if( maxInlierSet.size() > pts.size()*0.6)
            break;
    }
    //cout<<"Lineextractor:bestA:"<<bestA.xyz[0]<<","<<bestA.xyz[1]<<endl;
    RandomLine3d rl;
    if (maxInlierSet.size() >= 2) {
        cv::Point3d m = (bestA.pos+bestB.pos)*0.5, d = bestB.pos-bestA.pos;
        // optimize and reselect inliers
        // compute a 3d line using algebraic method
        while(true) {
            vector<int> tmpInlierSet;
            cv::Point3d tmp_m, tmp_d;
            computeLine3d_svd(pts,maxInlierSet, tmp_m, tmp_d);
            for(int i=0; i<pts.size(); ++i) {
                if(mah_dist3d_pt_line(pts[i], tmp_m, tmp_m+tmp_d) < distThresh) {
                    tmpInlierSet.push_back(i);
                }
            }
            if(tmpInlierSet.size() > maxInlierSet.size()) {
                maxInlierSet = tmpInlierSet;
                m = tmp_m;
                d = tmp_d;
            } else
                break;
        }
        // find out two endpoints
        double minv=100, maxv=-100;
        int	   idx_end1 = 0, idx_end2 = 0;
        for(int i=0; i<maxInlierSet.size(); ++i) {
            double dproduct = (pts[maxInlierSet[i]].pos-m).dot(d);
            if ( dproduct < minv) {
                minv = dproduct;
                idx_end1 = i;
            }
            if (dproduct > maxv) {
                maxv = dproduct;
                idx_end2 = i;
            }
        }
        rl.A = pts[maxInlierSet[idx_end1]].pos;
        rl.B = pts[maxInlierSet[idx_end2]].pos;
    }
    rl.director=rl.A-rl.B;
    rl.mid=(rl.A+rl.B)/2;
    //cout<<"Lineextractor:rl.A:"<<rl.A.x<<", "<<rl.A.y<<", "<<rl.A.z<<endl;
    rl.pts.resize(maxInlierSet.size());
    for(int i=0; i< maxInlierSet.size();++i) rl.pts[i]= pts[maxInlierSet[i]] ;
    return rl;
}

bool verify3dLine(const vector<RandomPoint3d>& pts, const cv::Point3d& A,  const cv::Point3d& B)
// input: line AB, collinear points
// output: whether AB is a good representation for points
// method: divide AB (or CD, which is endpoints of the projected points on AB)
// into n sub-segments, detect how many sub-segments containing
// at least one point(projected onto AB), if too few, then it implies invalid line
{
    int nCells = 10;//sysPara.num_cells_lineseg_range; // number of cells
    int* cells = new int[nCells];
    double ratio = 0.7;//sysPara.ratio_support_pts_on_line;
    for(int i=0; i<nCells; ++i) cells[i] = 0;
    int nPts = pts.size();
    // find 2 extremities of points along the line direction
    double minv=100, maxv=-100;
    int	   idx1 = 0, idx2 = 0;
    for(int i=0; i<nPts; ++i) {
        if ((pts[i].pos-A).dot(B-A) < minv) {
            minv = (pts[i].pos-A).dot(B-A);
            idx1 = i;
        }
        if ((pts[i].pos-A).dot(B-A) > maxv) {
            maxv = (pts[i].pos-A).dot(B-A);
            idx2 = i;
        }
    }
    cv::Point3d C = projectPt3d2Ln3d (pts[idx1].pos, (A+B)*0.5, B-A);
    cv::Point3d D = projectPt3d2Ln3d (pts[idx2].pos, (A+B)*0.5, B-A);
    double cd = cv::norm(D-C);
    if(cd < EPS) {
        delete[] cells;
        return false;
    }
    for(int i=0; i<nPts; ++i) {
        cv::Point3d X = pts[i].pos;
        double lambda = abs((X-C).dot(D-C)/cd/cd); // 0 <= lambd <=1
        if (lambda>=1) {
            cells[nCells-1] += 1;
        } else {
            cells[(unsigned int)floor(lambda*10)] += 1;
        }
    }
    double sum = 0;
    for (int i=0; i<nCells; ++i) {
        //		cout<<cells[i]<<"\t";
        if (cells[i] > 0 )
            sum=sum+1;
    }

    delete[] cells;
    if(sum/nCells > ratio) {
        return true;
    } else {
        return false;
    }
}


double mah_dist3d_pt_line (const RandomPoint3d& pt, const cv::Point3d& q1, const cv::Point3d& q2)
// compute the Mahalanobis distance between a random 3d point p and line (q1,q2)
// this is fater version since the point cov has already been decomposed by svd
{
    if (pt.U.cols != 3)	{
        cerr<<"Error in mah_dist3d_pt_line: R matrix must be 3x3"<<endl;
        return -1;
    }
    double out;
/*	double s0 = pt.W_sqrt[0], s1 = pt.W_sqrt[1], s2 = pt.W_sqrt[2];
	double r11, r12, r13, r21, r22, r23, r31, r32, r33;
	r11 = pt.U.at<double>(0,0);
	r12 = pt.U.at<double>(0,1);
	r13 = pt.U.at<double>(0,2);
	r21 = pt.U.at<double>(1,0);
	r22 = pt.U.at<double>(1,1);
	r23 = pt.U.at<double>(1,2);
	r31 = pt.U.at<double>(2,0);
	r32 = pt.U.at<double>(2,1);
	r33 = pt.U.at<double>(2,2);

	cv::Point3d q1_p = q1 - pt.pos, q2_p = q2 - pt.pos;

	cv::Point3d q1n((q1_p.x * r11 + q1_p.y * r21 + q1_p.z * r31)/s0,
		(q1_p.x * r12 + q1_p.y * r22 + q1_p.z * r32)/s1,
		(q1_p.x * r13 + q1_p.y * r23 + q1_p.z * r33)/s2),
		q2n((q2_p.x * r11 + q2_p.y * r21 + q2_p.z * r31)/s0,
		(q2_p.x * r12 + q2_p.y * r22 + q2_p.z * r32)/s1,
		(q2_p.x * r13 + q2_p.y * r23 + q2_p.z * r33)/s2);
	 out = cv::norm(q1n.cross(q2n))/cv::norm(q1n-q2n);
*/

    double xa = q1.x, ya = q1.y, za = q1.z;
    double xb = q2.x, yb = q2.y, zb = q2.z;
    double c1 = pt.DU[0], c2 = pt.DU[1], c3 = pt.DU[2],
            c4 = pt.DU[3], c5 = pt.DU[4], c6 = pt.DU[5],
            c7 = pt.DU[6], c8 = pt.DU[7], c9 = pt.DU[8];
    //cout<<"Lineextractor:c1,c2:"<<c1<<","<<c2<<endl;
    double x1 = pt.pos.x, x2 = pt.pos.y, x3 = pt.pos.z;
    double term1 = ((c1*(x1-xa)+c2*(x2-ya)+c3*(x3-za))*(c4*(x1-xb)+c5*(x2-yb)+c6*(x3-zb))
                    -(c4*(x1-xa)+c5*(x2-ya)+c6*(x3-za))*(c1*(x1-xb)+c2*(x2-yb)+c3*(x3-zb))),
            term2 = ((c1*(x1-xa)+c2*(x2-ya)+c3*(x3-za))*(c7*(x1-xb)+c8*(x2-yb)+c9*(x3-zb))
                     -(c7*(x1-xa)+c8*(x2-ya)+c9*(x3-za))*(c1*(x1-xb)+c2*(x2-yb)+c3*(x3-zb))),
            term3 = ((c4*(x1-xa)+c5*(x2-ya)+c6*(x3-za))*(c7*(x1-xb)+c8*(x2-yb)+c9*(x3-zb))
                     -(c7*(x1-xa)+c8*(x2-ya)+c9*(x3-za))*(c4*(x1-xb)+c5*(x2-yb)+c6*(x3-zb))),
            term4 = (c1*(x1-xa)-c1*(x1-xb)+c2*(x2-ya)-c2*(x2-yb)+c3*(x3-za)-c3*(x3-zb)),
            term5 = (c4*(x1-xa)-c4*(x1-xb)+c5*(x2-ya)-c5*(x2-yb)+c6*(x3-za)-c6*(x3-zb)) ,
            term6 = (c7*(x1-xa)-c7*(x1-xb)+c8*(x2-ya)-c8*(x2-yb)+c9*(x3-za)-c9*(x3-zb));
    out = sqrt((term1 * term1 + term2* term2 + term3 * term3)/( term4*term4 + term5*term5 + term6*term6));
/*
    double x1 = pt.dux[0], x2 = pt.dux[1], x3 = pt.dux[2];
    double term1 = (-x1+c1*xa+c2*ya+c3*za)*(-x2+c4*xb+c5*yb+c6*zb)-(-x2+c4*xa+c5*ya+c6*za)*(-x1+c1*xb+c2*yb+c3*zb),
     term2 =(-x1+c1*xa+c2*ya+c3*za)*(-x3+c7*xb+c8*yb+c9*zb)-(-x3+c7*xa+c8*ya+c9*za)*(-x1+c1*xb+c2*yb+c3*zb),
     term3 = (-x2+c4*xa+c5*ya+c6*za)*(-x3+c7*xb+c8*yb+c9*zb)-(-x3+c7*xa+c8*ya+c9*za)*(-x2+c4*xb+c5*yb+c6*zb),
     term4 = c1*xa-c1*xb+c2*ya-c2*yb+c3*za-c3*zb,
     term5 = c4*xa-c4*xb+c5*ya-c5*yb+c6*za-c6*zb,
     term6 = c7*xa-c7*xb+c8*ya-c8*yb+c9*za-c9*zb;
    out = sqrt(((term1*term1)+(term2*term2)+(term3*term3))/((term4*term4)+(term5*term5)+(term6*term6)));
*/
    return out;

}
cv::Mat q2r (double* q)
// input: unit quaternion representing rotation
// output: 3x3 rotation matrix
// note: q=(a,b,c,d)=a + b i + c j + d k, where (b,c,d) is the rotation axis
{
    double  a = q[0],	b = q[1],
            c = q[2],	d = q[3];
    double nm = sqrt(a*a+b*b+c*c+d*d);
    a = a/nm;
    b = b/nm;
    c = c/nm;
    d = d/nm;
    cv::Mat R = (cv::Mat_<double>(3,3)<<
                                      a*a+b*b-c*c-d*d,	2*b*c-2*a*d,		2*b*d+2*a*c,
            2*b*c+2*a*d,		a*a-b*b+c*c-d*d,	2*c*d-2*a*b,
            2*b*d-2*a*c,		2*c*d+2*a*b,		a*a-b*b-c*c+d*d);
    return R.clone();
}
void costFun_optimizeRelmotion(double *p, double *error, int m, int n, void *adata)
{
//	MyTimer t; t.start();
    struct Data_optimizeRelmotion* dptr;
    dptr = (struct Data_optimizeRelmotion *) adata;
    cv::Mat R = q2r(p);
    cv::Mat t = cv::Mat(3,1,CV_64F, &p[4]);// (cv::Mat_<double>(3,1)<<p[4],p[5],p[6]);
    double cost = 0;
#pragma omp  parallel for
    for(int i=0; i< dptr->a.size() ; ++i)	{
#ifdef OPT_USE_MAHDIST
        /*	error[i] = 0.25*(mah_dist3d_pt_line(dptr->b[i].rndA, R*cvpt2mat(dptr->a[i].A,0)+t, R*cvpt2mat(dptr->a[i].B,0)+t)+
				   mah_dist3d_pt_line(dptr->b[i].rndB, R*cvpt2mat(dptr->a[i].A,0)+t, R*cvpt2mat(dptr->a[i].B,0)+t)+
				   mah_dist3d_pt_line(dptr->a[i].rndA, R.t()*(cvpt2mat(dptr->b[i].A,0)-t), R.t()*(cvpt2mat(dptr->b[i].B,0)-t))+
				   mah_dist3d_pt_line(dptr->a[i].rndB, R.t()*(cvpt2mat(dptr->b[i].A,0)-t), R.t()*(cvpt2mat(dptr->b[i].B,0)-t)));
	*/
		// faster computing than above
		double aiA[3] = {dptr->a[i].A.x,dptr->a[i].A.y,dptr->a[i].A.z},
			aiB[3] = {dptr->a[i].B.x,dptr->a[i].B.y,dptr->a[i].B.z},
			biA[3] = {dptr->b[i].A.x,dptr->b[i].A.y,dptr->b[i].A.z},
			biB[3] = {dptr->b[i].B.x,dptr->b[i].B.y,dptr->b[i].B.z};
		error[i] = 0.25*(mah_dist3d_pt_line(dptr->b[i].rndA, R*array2mat(aiA,3)+t, R*array2mat(aiB,3)+t)+
				   mah_dist3d_pt_line(dptr->b[i].rndB, R*array2mat(aiA,3)+t, R*array2mat(aiB,3)+t)+
				   mah_dist3d_pt_line(dptr->a[i].rndA, R.t()*(array2mat(biA,3)-t), R.t()*(array2mat(biB,3)-t))+
				   mah_dist3d_pt_line(dptr->a[i].rndB, R.t()*(array2mat(biA,3)-t), R.t()*(array2mat(biB,3)-t)));

#else
        error[i]= 0.25*(dist3d_pt_line(dptr->b[i].A, mat2cvpt3d(R*cvpt2mat(dptr->a[i].A,0)+t), mat2cvpt3d(R*cvpt2mat(dptr->a[i].B,0)+t))
                        + dist3d_pt_line(dptr->b[i].B, mat2cvpt3d(R*cvpt2mat(dptr->a[i].A,0)+t), mat2cvpt3d(R*cvpt2mat(dptr->a[i].B,0)+t))
                        + dist3d_pt_line(dptr->a[i].A, mat2cvpt3d(R.t()*(cvpt2mat(dptr->b[i].A,0)-t)), mat2cvpt3d(R.t()*(cvpt2mat(dptr->b[i].B,0)-t)))
                        + dist3d_pt_line(dptr->a[i].B, mat2cvpt3d(R.t()*(cvpt2mat(dptr->b[i].A,0)-t)), mat2cvpt3d(R.t()*(cvpt2mat(dptr->b[i].B,0)-t))));

#endif
    }
}




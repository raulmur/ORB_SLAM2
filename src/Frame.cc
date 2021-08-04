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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>
#include <include/LocalMapping.h>
namespace ORB_SLAM2
{

    long unsigned int Frame::nNextId=0;
    bool Frame::mbInitialComputations=true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

    Frame::Frame()
    {}

//Copy Constructor
    Frame::Frame(const Frame &frame)
            :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
             mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
             mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
             mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
             mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
             mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
             mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
             mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
             mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
             mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
             mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
             mLdesc(frame.mLdesc), NL(frame.NL), mvKeylinesUn(frame.mvKeylinesUn), mvpMapLines(frame.mvpMapLines),  //线特征相关的类成员变量
             mvbLineOutlier(frame.mvbLineOutlier), mvKeyLineFunctions(frame.mvKeyLineFunctions),mvDepthLine(frame.mvDepthLine),
             mvLines3D(frame.mvLines3D),mv3DLineforMap(frame.mv3DLineforMap),dealWithLine(frame.dealWithLine),blurNumber(frame.blurNumber),vSurfaceNormal(frame.vSurfaceNormal),
             vVanishingDirection(frame.vVanishingDirection),mVF3DLines(frame.mVF3DLines), vSurfaceNormalx(frame.vSurfaceNormalx)
    {
        for(int i=0;i<FRAME_GRID_COLS;i++)
            for(int j=0; j<FRAME_GRID_ROWS; j++)
                mGrid[i][j]=frame.mGrid[i][j];

        if(!frame.mTcw.empty())
            SetPose(frame.mTcw);
    }


    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
            :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
             mpReferenceKF(static_cast<KeyFrame*>(NULL))
    {
        // Frame ID
        mnId=nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
        thread threadRight(&Frame::ExtractORB,this,1,imRight);
        threadLeft.join();
        threadRight.join();

        N = mvKeys.size();

        if(mvKeys.empty())
            return;

        UndistortKeyPoints();

        ComputeStereoMatches();

        mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        mvbOutlier = vector<bool>(N,false);


        // This is done only for the first Frame (or after a change in the calibration)
        if(mbInitialComputations)
        {
            ComputeImageBounds(imLeft);

            mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
            mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

            fx = K.at<float>(0,0);
            fy = K.at<float>(1,1);
            cx = K.at<float>(0,2);
            cy = K.at<float>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mbInitialComputations=false;
        }

        mb = mbf/fx;

        AssignFeaturesToGrid();
    }

//RGB-D 初始化frame
    Frame::Frame(const cv::Mat &imGray,const double &timeStamp, const cv::Mat &imDepth,  ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
            :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
             mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
    {
        // Frame ID
        mnId=nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        cv::Mat tmpK = (cv::Mat_<double>(3,3)<< fx, 0, cx,
                0, fy, cy,
                0,0,1);
        dealWithLine=false;//默认情况下，不使用线特征

        ExtractORB(0,imGray);

        N = mvKeys.size();
        //NL=0;//没有线特征
        featureSelect(imGray);
        cout<<"dealwithline frame"<<dealWithLine<<endl;

        NL=0;
        if(dealWithLine)
        {
            ExtractLSD(imGray);
            //cout<<"Frame: number of lines"<<mvKeylinesUn.size()<<endl;
            isLineGood(imGray,imDepth,tmpK);
            NL=mvKeylinesUn.size();
            cout<<"Frame, NL"<<NL<<endl;
        }


        if(mvKeys.empty())
            return;

        UndistortKeyPoints();

        ComputeStereoFromRGBD(imDepth);

        mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        mvpMapLines=vector<MapLine *>(NL,static_cast<MapLine*>(NULL));
        mvbOutlier = vector<bool>(N,false);
        mvbLineOutlier=vector<bool>(NL,false);



        // This is done only for the first Frame (or after a change in the calibration)
        if(mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
            mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

            fx = K.at<float>(0,0);
            fy = K.at<float>(1,1);
            cx = K.at<float>(0,2);
            cy = K.at<float>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mbInitialComputations=false;
        }

        mb = mbf/fx;

        AssignFeaturesToGrid();
    }


    Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
            :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
             mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
    {
        mnId=nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB and LSD extraction
        ExtractORB(0,imGray);
        ExtractLSD(imGray);
        dealWithLine = true;
        N = mvKeys.size();
        NL = mvKeylinesUn.size();

        if(mvKeys.empty())
            return;

        UndistortKeyPoints();


        //TODO 获取surface normal的信息
        //cout<<timeStamp<<"/home/yan/Documents/1_research/Manhanttan-master/ICL-NUIMdataset/lr_k0/normals/"+std::to_string(int(timeStamp))+".png"<<endl;
        //cv::Mat imNormal=cv::imread("/home/yan/Documents/1_research/Manhanttan-master/ICL-NUIMdataset/lr_k0/normals/"+std::to_string(int(timeStamp))+".png");
        //cv::Mat imNormal=cv::imread("/home/yan/Documents/1_research/2_cnn-slam/DataProcess/normals_stfar_exr/"+std::to_string(mTimeStamp)+".exr", cv::IMREAD_UNCHANGED);

        //cout<< imNormal<<endl;
        //cv::imwrite("normal.png",imNormal);



        //TODO 计算 vanishing direction of lines


        // Set no stereo information
        mvuRight = vector<float>(N,-1);
        mvDepth = vector<float>(N,-1);

        mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        mvbOutlier = vector<bool>(N,false);

        mvpMapLines = vector<MapLine*>(NL,static_cast<MapLine*>(NULL));
        mvbLineOutlier = vector<bool>(NL,false);

        // This is done only for the first Frame (or after a change in the calibration)
        if(mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
            mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

            fx = K.at<float>(0,0);
            fy = K.at<float>(1,1);
            cx = K.at<float>(0,2);
            cy = K.at<float>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mbInitialComputations=false;
        }

        mb = mbf/fx;

        AssignFeaturesToGrid();
    }

    // monocular with normal
Frame::Frame(const cv::Mat &imGray,const cv::Mat &imNormal, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
        :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
         mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB and LSD extraction
    ExtractORB(0,imGray);
    ExtractLSD(imGray);
    dealWithLine = true;
    N = mvKeys.size();
    NL = mvKeylinesUn.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();


    //TODO 获取surface normal的信息
    //cout<<timeStamp<<"/home/yan/Documents/1_research/Manhanttan-master/ICL-NUIMdataset/lr_k0/normals/"+std::to_string(int(timeStamp))+".png"<<endl;
    //cv::Mat imNormal=cv::imread("/home/yan/Documents/1_research/Manhanttan-master/ICL-NUIMdataset/lr_k0/normals/"+std::to_string(int(timeStamp))+".png");
    //cv::Mat imNormal=cv::imread("/home/yan/Documents/1_research/2_cnn-slam/DataProcess/normals_stfar_exr/"+std::to_string(mTimeStamp)+".exr", cv::IMREAD_UNCHANGED);

    //cout<< imNormal<<endl;
    //cv::imwrite("normal.png",imNormal);

    if(imNormal.empty())
    {
        /*cerr << endl << "Failed to load image at: "
              << endl;*/

    }

    AnalysisSurfaceNormalfromImage( imNormal, K);

    //TODO 计算 vanishing direction of lines


    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    mvpMapLines = vector<MapLine*>(NL,static_cast<MapLine*>(NULL));
    mvbLineOutlier = vector<bool>(NL,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}



void Frame::AnalysisSurfaceNormalfromImage(const cv::Mat &imNormal, cv::Mat &K)
{
    //imNormal
    float x_ratio=0.0; float y_ratio=0.0;
    vector<SurfaceNormal> surfaceNormalVector;
    surfaceNormalVector.reserve(imNormal.rows * imNormal.cols);
    x_ratio=640.0/imNormal.cols;
    y_ratio=480.0/imNormal.rows;
    for (int i = 0; i < imNormal.rows; i++)
    {
        for (int j = 0; j < imNormal.cols; j++)
        {
            //彩色图像获取到单个像素
            //Vec3f pix = imNormal.at<Vec3f>(i,j).val;
            cv::Point3f normalVector;
            //获取到RGB分量的值。

//            normalVector.x= imNormal.at<Vec3f>(i,j).val[0];         //b
//            normalVector.y = imNormal.at<Vec3f>(i,j).val[1];        //g
//            normalVector.z = imNormal.at<Vec3f>(i,j).val[2];///128-1;        //r
            //cout<<"Frame: normal"<<imNormal.at<Vec3f>(i,j)<<", "<<normalVector.x<<", "<<normalVector.y<<", "<<normalVector.z<<endl;
            //或者使用下面的方法
            //uchar B = img.at<Vec3b>(i, j)[0];
            //uchar G = img.at<Vec3b>(i, j)[1];
            //uchar G = img.at<Vec3b>(i, j)[2];
            //计算灰度值，然后赋值给灰度图中的像素
            normalVector.x= (imNormal.at<Vec3b>(i,j).val[0]/128.0)-1;         //b
            normalVector.y = (imNormal.at<Vec3b>(i,j).val[1]/128.0)-1;        //g
            normalVector.z = (imNormal.at<Vec3b>(i,j).val[2]/128.0)-1;///128-1;

            cv::Point3f normalVectort=normalVector/norm(normalVector);
            surfacenomal.normal=normalVectort;
            surfacenomal.FramePosition=cv::Point2i(j*x_ratio,i*y_ratio);
            surfaceNormalVector.push_back(surfacenomal);
        }
    }


    vSurfaceNormal=surfaceNormalVector;
}

    void Frame::AssignFeaturesToGrid()
    {
        int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
        for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
                mGrid[i][j].reserve(nReserve);

        for(int i=0;i<N;i++)
        {
            const cv::KeyPoint &kp = mvKeysUn[i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }
// line feature extractor, 自己添加的
    void Frame::ExtractLSD(const cv::Mat &im)
    {
        mpLineSegment->ExtractLineSegment(im, mvKeylinesUn, mLdesc, mvKeyLineFunctions);
        //选择一部分 直线 ，由于3D点 的问题
        //cout<<"Frame:the lines of "<<mvKeylinesUn.size()<<endl;

    }

    void Frame::ExtractORB(int flag, const cv::Mat &im)
    {
        if(flag==0)
            (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
        else
            (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
    }

    void Frame::featureSelect(const cv::Mat &im)
    {
        //
        Mat gray1;
        double m1,sd1;
        Mat tmp_m1,tmp_sd1;
        Laplacian(im, gray1, CV_16S,3);
        //计算均值和方差
        meanStdDev(gray1,tmp_m1, tmp_sd1);
        m1 = tmp_m1.at<double>(0, 0);		//均值
        sd1 = tmp_sd1.at<double>(0, 0);		//标准差
        //cout << "原图像：" << endl;
        //cout << "均值: " << m1 << " , 方差: " << sd1<< endl;
        blurNumber=sd1*sd1;
        if (sd1*sd1 < 1200 || N<50)
        {
            cout << "blur,sd1^2="<<sd1*sd1 << endl;
            dealWithLine=true;
        }
        else
        {
            cout << "not blur,sd1^2="<<sd1*sd1 << endl;
        }


    }

    void Frame::isLineGood(const cv::Mat &imGray,  const cv::Mat &imDepth,cv::Mat K)
    {
        mvDepthLine = vector<float>(mvKeylinesUn.size(),-1.0f);
        mvLines3D=vector<Vector6d>(mvKeylinesUn.size(), static_cast<Vector6d>(NULL));
        ofstream origPoints("../data/origpoints.txt",ios::app);
        ofstream optiPoints("../data/outpoints.txt",ios::app);
        ofstream origPoints_xyzrgb("orig_xyzrgb.txt",ios::app);
        for(int i=0; i<mvKeylinesUn.size(); ++i)	{ // each line
            double len = cv::norm(mvKeylinesUn[i].getStartPoint() - mvKeylinesUn[i].getEndPoint());
            //cout<<"Len:"<<len<<"Posi a:"<<mvKeylinesUn[i].getStartPoint().x<<"Posi b:"<<mvKeylinesUn[i].getStartPoint().y<<endl;
            //cout<<"Len:"<<len<<"Posi a:"<<mvKeylinesUn[i].getEndPoint().x<<"Posi b:"<<mvKeylinesUn[i].getEndPoint().y<<endl;

            vector<cv::Point3d> pts3d;
            //vector<pcl::PointXYZRGBA> vline;
            // iterate through a line
            double numSmp = (double) min((int)len, 100); //number of line points sampled

            //cout<<"numSmp:"<<numSmp<<endl;

            pts3d.reserve(numSmp);//预留数量

            for(int j=0; j<=numSmp; ++j) {
                // use nearest neighbor to querry depth value
                // assuming position (0,0) is the top-left corner of image, then the
                // top-left pixel's center would be (0.5,0.5)
                cv::Point2d pt = mvKeylinesUn[i].getStartPoint() * (1-j/numSmp) + mvKeylinesUn[i].getEndPoint()* (j/numSmp);

                //cout<<"pt"<<pt.x<<","<<pt.y<<endl;
                if(pt.x<0 || pt.y<0 || pt.x >= imDepth.cols || pt.y >= imDepth.rows ) continue;
                int row, col; // nearest pixel for pt
                if((floor(pt.x) == pt.x) && (floor(pt.y) == pt.y)) {// boundary issue
                    col = max(int(pt.x-1),0);
                    row = max(int(pt.y-1),0);
                } else {
                    col = int(pt.x);
                    row = int(pt.y);
                }
                //cout<<"col:"<<col<<","<<"row:"<<row<<endl;
                float d=-1;
                if(imDepth.at<float>(row,col) <=0.01) { // no depth info
                    //cout<<"no depth info"<<endl;
                    continue;
                }
                else {
                    d = imDepth.ptr<float>(row)[col];
                }
                cv::Point3d p;

                //cout<<"col:"<<col<<", row:"<<row<<endl;
                //cout<<"cx:"<<cx<<",fx"<<fx<<endl;
                // 计算这个点的空间坐标
                p.z = d;
                p.x = (col - cx) * p.z *invfx;//K.at<float>(0,0);//invfx;
                p.y = (row - cy) * p.z *invfy;//K.at<float>(1,1);//invfy;
                //cout<<"x:"<<p.x<<",y:"<<p.y<<",z:"<<p.z<<endl;

                //mv3DLineforMap.push_back(p);
                //origPoints_xyzrgb<<p.x<<" "<<p.y<<" "<<p.z<<endl;
                pts3d.push_back(p);

            }
            //如果 点数量少于 10或者 ，就抛弃该线
            if (pts3d.size() < 10.0)//sysPara.ratio_of_collinear_pts
                continue;

            RandomLine3d tmpLine;
#if 1
            vector<RandomPoint3d> rndpts3d;
            rndpts3d.reserve(pts3d.size());
            //cout<<"pts3d size:"<<pts3d.size()<<endl;
            // compute uncertainty of 3d points
            for(int j=0; j<pts3d.size();++j) {
                //cout<<"ptsxyz"<<pts3d[j].x<<","<<pts3d[j].y<<","<<pts3d[j].z<<endl;
                rndpts3d.push_back(compPt3dCov(pts3d[j], K, 1));
                //cout<<"Du[0]:"<<compPt3dCov(pts3d[j], mK, 1).DU[0]<<endl;
            }
            // using ransac to extract a 3d line from 3d pts
            tmpLine = extract3dline_mahdist(rndpts3d);
#else
            //计算这个线的参数
        tmpLine = extract3dline(pts3d,origPoints,optiPoints);
#endif
            //cout<<"jieshu"<<endl;
            //修复这条直线上的点


            if(tmpLine.pts.size()/len > 0.4/*sysPara.ratio_of_collinear_pts*/	&&
               cv::norm(tmpLine.A - tmpLine.B) >0.02) { // sysPara.line3d_length_thresh
                //this line is reliable
                mvDepthLine[i]= 1.0f;
                //cout<<"gaibian line 3d"<<endl;
                mvLines3D[i]<<tmpLine.A.x,tmpLine.A.y,tmpLine.A.z,tmpLine.B.x,tmpLine.B.y,tmpLine.B.z;
            }

            if(1==mvDepthLine[i])
            {
                // rebuild this line in depth map

                for(int j=0; j<=numSmp; ++j)
                {
                    cv::Point2d pt = mvKeylinesUn[i].getStartPoint() * (1-j/numSmp) + mvKeylinesUn[i].getEndPoint() * (j/numSmp);
                    if(pt.x<0 || pt.y<0 || pt.x >= imDepth.cols || pt.y >= imDepth.rows ) continue;

                    int row, col; // nearest pixel for pt
                    if((floor(pt.x) == pt.x) && (floor(pt.y) == pt.y)) {// boundary issue
                        col = max(int(pt.x-1),0);
                        row = max(int(pt.y-1),0);
                    }
                    else {
                        col = int(pt.x);
                        row = int(pt.y);
                    }


                    float d=imDepth.ptr<float>(row)[col];
                    cv::Point3d p;
                    if(d<=0.01) continue;
                    else
                    {
                        p.z=d;
                        if (p.z < 0.01 || p.z>10){cout<<"Frame: invalid p.z"<<endl;continue;}

                        p.x = (col - cx) * p.z *invfx;
                        p.y = (row - cy) * p.z *invfy;
                        p.z=fy*fx*(tmpLine.director.y*tmpLine.mid.x-
                                   tmpLine.director.x*tmpLine.mid.y)/
                            (fy*tmpLine.director.y*(col-cx)-
                             fx*tmpLine.director.x*(row-cy));
                        //cout<<p.x<<", "<<p.y<<", "<<p.z<<endl;
                        mv3DLineforMap.push_back(p);
                    }
                    //tmpLine.pts.push_back(p);


                }
            }
        }
        return;
    }

    void Frame::isLineGood(const cv::Mat &imGray, const cv::Mat &imDepth)
    {
        mvDepthLine = vector<float>(mvKeylinesUn.size(),-1.0f);
        mvLines3D=vector<Vector6d>(mvKeylinesUn.size(), static_cast<Vector6d>(NULL));
        ofstream origPoints("../data/origpoints.txt",ios::app);
        ofstream optiPoints("../data/outpoints.txt",ios::app);
        ofstream origPoints_xyzrgb("../data/orig_xyzrgb.txt",ios::app);
        for(int i=0; i<mvKeylinesUn.size(); ++i)	{ // each line
            double len = cv::norm(mvKeylinesUn[i].getStartPoint() - mvKeylinesUn[i].getEndPoint());
            //cout<<"Len:"<<len<<"Posi a:"<<mvKeylinesUn[i].getStartPoint()<<"Posi b:"<<mvKeylinesUn[i].getEndPoint()<<endl;

            vector<cv::Point3d> pts3d;
            //vector<pcl::PointXYZRGBA> vline;
            // iterate through a line
            double numSmp = (double) min((int)len, 100); // number of line points sampled

            //cout<<"numSmp:"<<numSmp<<endl;

            pts3d.reserve(numSmp);//预留数量

            for(int j=0; j<=numSmp; ++j) {
                // use nearest neighbor to querry depth value
                // assuming position (0,0) is the top-left corner of image, then the
                // top-left pixel's center would be (0.5,0.5)
                cv::Point2d pt = mvKeylinesUn[i].getStartPoint() * (1-j/numSmp) + mvKeylinesUn[i].getEndPoint()* (j/numSmp);
                if(pt.x<0 || pt.y<0 || pt.x >= imDepth.cols || pt.y >= imDepth.rows ) continue;
                int row, col; // nearest pixel for pt
                if((floor(pt.x) == pt.x) && (floor(pt.y) == pt.y)) {// boundary issue
                    col = max(int(pt.x-1),0);
                    row = max(int(pt.y-1),0);
                } else {
                    col = int(pt.x);
                    row = int(pt.y);
                }


                ushort d=-1;
                if(imDepth.at<ushort>(row,col) <=0) { // no depth info
                    //cout<<"no depth info"<<endl;
                    continue;
                }
                else {
                    d = imDepth.ptr<ushort>(row)[col];
                }
                cv::Point3d p;

                //cout<<"col:"<<col<<", row:"<<row<<endl;
                //cout<<"cx:"<<cx<<",fx"<<fx<<endl;
                // 计算这个点的空间坐标
                p.z = double(d);
                p.x = (col - cx) * p.z / fx;
                p.y = (row - cy) * p.z / fy;
                //cout<<"x:"<<p.x<<",y:"<<p.y<<",z:"<<p.z<<endl;


                //origPoints_xyzrgb<<p.x<<" "<<p.y<<" "<<p.z<<endl;
                pts3d.push_back(p);

            }
            //如果 点数量少于 10或者 ，就抛弃该线
            if (pts3d.size() < 10.0)//sysPara.ratio_of_collinear_pts
                continue;

            RandomLine3d tmpLine;

            //计算这个线的参数
            tmpLine = extract3dline(pts3d,origPoints,optiPoints);
            //cout<<"jieshu"<<endl;
            //修复这条直线上的点
            /*
             for(int j=0; j<=numSmp; ++j)
             {
                 cv::Point2d pt = mvKeylinesUn[i].getStartPoint() * (1-j/numSmp) + mvKeylinesUn[i].getEndPoint() * (j/numSmp);
                 if(pt.x<0 || pt.y<0 || pt.x >= imDepth.cols || pt.y >= imDepth.rows ) continue;

                 int row, col; // nearest pixel for pt
                 if((floor(pt.x) == pt.x) && (floor(pt.y) == pt.y)) {// boundary issue
                     col = max(int(pt.x-1),0);
                     row = max(int(pt.y-1),0);
                 } else {
                     col = int(pt.x);
                     row = int(pt.y);
                 }


                 double z=0;
                 cv::Point3d p;
                 if(imDepth.at<ushort>(row,col) <=0) continue;
                 else
                 {
                     ushort d = imDepth.ptr<ushort>(row)[col];
                     //cv::Point3d p;
                     double origi_z=0;
                     origi_z= double(d) / 5000.0;
                     // 计算这个点的空间坐标
                     p.z=fy*fx*(tmpLine.director.y*tmpLine.A.x-
                                tmpLine.director.x*tmpLine.A.y)/
                         (fy*tmpLine.director.y*(cx)-
                          fx*tmpLine.director.x*(row-cy));
                     p.x = (col - cx) * p.z / fx;
                     p.y = (row - cy) * p.z / fy;
                     //cout<<"origi_z"<<origi_z<<","<<"oti_z"<<p.z<<endl;


                 }
                 tmpLine.pts.push_back(p);
     //                cv::Point3d m= (tmpLine.A+tmpLine.B)/2;
     //                if(tmpLine.director.x!=0)
     //                    z = (p.x - m.x) * tmpLine.director.z / tmpLine.director.x + m.z;
     //                else
     //                    z= (p.y-m.y)*tmpLine.director.z/tmpLine.director.y+m.z;
     ////
                // optiPoints<<p.x<<","<<p.y<<","<<p.z<<endl;

             }*/

            if(tmpLine.pts.size()/len > 0.4/*sysPara.ratio_of_collinear_pts*/	&&
               cv::norm(tmpLine.A - tmpLine.B) >0.02) { // sysPara.line3d_length_thresh
                //MLEstimateLine3d (tmpLine, 100);
                //判断深度是否存在
                //cout<<"gaibian depth line"<<endl;
                cout<<mvDepthLine[i]<<endl;
                mvDepthLine[i]= 1.0f;
                //cout<<"gaibian line 3d"<<endl;
                mvLines3D[i]<<tmpLine.A.x,tmpLine.A.y,tmpLine.A.z,tmpLine.B.x,tmpLine.B.y,tmpLine.B.z;

            }
            //cout<<"jieshu  gaibian"<<endl;


        }
        return;

    }
// 根据两个匹配的特征线计算特征线的3D坐标, frame1是当前帧，frame2是前一帧

// line descriptor MAD, 自己添加的
    void Frame::lineDescriptorMAD( vector<vector<DMatch>> line_matches, double &nn_mad, double &nn12_mad) const
    {
        vector<vector<DMatch>> matches_nn, matches_12;
        matches_nn = line_matches;
        matches_12 = line_matches;
//    cout << "Frame::lineDescriptorMAD——matches_nn = "<<matches_nn.size() << endl;

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



    void Frame::SetPose(cv::Mat Tcw)
    {
        mTcw = Tcw.clone();
        UpdatePoseMatrices();
    }

    void Frame::UpdatePoseMatrices()
    {
        mRcw = mTcw.rowRange(0,3).colRange(0,3);
        mRwc = mRcw.t();
        mtcw = mTcw.rowRange(0,3).col(3);
        mOw = -mRcw.t()*mtcw;
    }

    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
    {
        pMP->mbTrackInView = false;

        // 3D in absolute coordinates
        cv::Mat P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const cv::Mat Pc = mRcw*P+mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY= Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if(PcZ<0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        if(u<mnMinX || u>mnMaxX)
            return false;
        if(v<mnMinY || v>mnMaxY)
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P-mOw;
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            return false;

        // Check viewing angle
        cv::Mat Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn)/dist;

        if(viewCos<viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist,this);

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = u;
        pMP->mTrackProjXR = u - mbf*invz;
        pMP->mTrackProjY = v;
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        return true;
    }

/**
 * @brief 判断MapLine的两个端点是否在视野内
 *
 * @param pML               MapLine
 * @param viewingCosLimit   视角和平均视角的方向阈值
 * @return                  true if the MapLine is in view
 */
    bool Frame::isInFrustum(MapLine *pML, float viewingCosLimit)
    {
        pML->mbTrackInView = false;

        Vector6d P = pML->GetWorldPos();

        cv::Mat SP = (Mat_<float>(3,1) << P(0), P(1), P(2));
        cv::Mat EP = (Mat_<float>(3,1) << P(3), P(4), P(5));

        // 两个端点在相机坐标系下的坐标
        const cv::Mat SPc = mRcw*SP + mtcw;
        const float &SPcX = SPc.at<float>(0);
        const float &SPcY = SPc.at<float>(1);
        const float &SPcZ = SPc.at<float>(2);

        const cv::Mat EPc = mRcw*EP + mtcw;
        const float &EPcX = EPc.at<float>(0);
        const float &EPcY = EPc.at<float>(1);
        const float &EPcZ = EPc.at<float>(2);

        // 检测两个端点的Z值是否为正
        if(SPcZ<0.0f || EPcZ<0.0f)
            return false;

        // V-D 1) 将端点投影到当前帧上，并判断是否在图像内
        const float invz1 = 1.0f/SPcZ;
        const float u1 = fx * SPcX * invz1 + cx;
        const float v1 = fy * SPcY * invz1 + cy;

        if(u1<mnMinX || u1>mnMaxX)
            return false;
        if(v1<mnMinY || v1>mnMaxY)
            return false;

        const float invz2 = 1.0f/EPcZ;
        const float u2 = fx*EPcX*invz2 + cx;
        const float v2 = fy*EPcY*invz2 + cy;

        if(u2<mnMinX || u2>mnMaxX)
            return false;
        if(v2<mnMinY || v2>mnMaxY)
            return false;

        // V-D 3)计算MapLine到相机中心的距离，并判断是否在尺度变化的距离内
        const float maxDistance = pML->GetMaxDistanceInvariance();
        const float minDistance = pML->GetMinDistanceInvariance();
        // 世界坐标系下，相机到线段中点的向量，向量方向由相机指向中点
        const cv::Mat OM = 0.5*(SP+EP) - mOw;
        const float dist = cv::norm(OM);

        if(dist<minDistance || dist>maxDistance)
            return false;

        // Check viewing angle
        // V-D 2)计算当前视角和平均视角夹角的余弦值，若小于cos(60°),即夹角大于60°则返回
        Vector3d Pn = pML->GetNormal();
        cv::Mat pn = (Mat_<float>(3,1) << Pn(0), Pn(1), Pn(2));
        const float viewCos = OM.dot(pn)/dist;

        if(viewCos<viewingCosLimit)
            return false;

        // Predict scale in the image
        // V-D 4) 根据深度预测尺度（对应特征在一层）
        const int nPredictedLevel = pML->PredictScale(dist, mfLogScaleFactor);

        // Data used by the tracking
        // 标记该特征将来要被投影
        pML->mbTrackInView = true;
        pML->mTrackProjX1 = u1;
        pML->mTrackProjY1 = v1;
        pML->mTrackProjX2 = u2;
        pML->mTrackProjY2 = v2;
        pML->mnTrackScaleLevel = nPredictedLevel;
        pML->mTrackViewCos = viewCos;

        return true;
    }


    vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
        if(nMinCellX>=FRAME_GRID_COLS)
            return vIndices;

        const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
        if(nMaxCellX<0)
            return vIndices;

        const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
        if(nMinCellY>=FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
        if(nMaxCellY<0)
            return vIndices;

        const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

        for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
        {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                if(vCell.empty())
                    continue;

                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    if(bCheckLevels)
                    {
                        if(kpUn.octave<minLevel)
                            continue;
                        if(maxLevel>=0)
                            if(kpUn.octave>maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    vector<size_t> Frame::GetLinesInArea(const float &x1, const float &y1, const float &x2, const float &y2, const float &r,
                                         const int minLevel, const int maxLevel) const
    {
        vector<size_t> vIndices;

        vector<KeyLine> vkl = this->mvKeylinesUn;

        const bool bCheckLevels = (minLevel>0) || (maxLevel>0);

//    for(vector<KeyLine>::iterator vit=vkl.begin(), vend=vkl.end(); vit!=vend; vit++)
        for(size_t i=0; i<vkl.size(); i++)
        {
            KeyLine keyline = vkl[i];

            // 1.对比中点距离
            float distance = (0.5*(x1+x2)-keyline.pt.x)*(0.5*(x1+x2)-keyline.pt.x)+(0.5*(y1+y2)-keyline.pt.y)*(0.5*(y1+y2)-keyline.pt.y);
            if(distance > r*r)
                continue;

            // 2.比较斜率，KeyLine的angle就是代表斜率
            float slope = (y1-y2)/(x1-x2)-keyline.angle;
            if(slope > r*0.01)
                continue;

            // 3.比较金字塔层数
            if(bCheckLevels)
            {
                if(keyline.octave<minLevel)
                    continue;
                if(maxLevel>=0 && keyline.octave>maxLevel)
                    continue;
            }

            vIndices.push_back(i);
        }

        return vIndices;
    }



    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
        posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            return false;

        return true;
    }


    void Frame::ComputeBoW()
    {
        if(mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
        }
    }

    void Frame::UndistortKeyPoints()
    {
        if(mDistCoef.at<float>(0)==0.0)
        {
            mvKeysUn=mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N,2,CV_32F);
        for(int i=0; i<N; i++)
        {
            mat.at<float>(i,0)=mvKeys[i].pt.x;
            mat.at<float>(i,1)=mvKeys[i].pt.y;
        }

        // Undistort points
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for(int i=0; i<N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x=mat.at<float>(i,0);
            kp.pt.y=mat.at<float>(i,1);
            mvKeysUn[i]=kp;
        }
    }

    void Frame::ComputeImageBounds(const cv::Mat &imLeft)
    {
        if(mDistCoef.at<float>(0)!=0.0)
        {
            cv::Mat mat(4,2,CV_32F);
            mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
            mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
            mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
            mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

            // Undistort corners
            mat=mat.reshape(2);
            cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
            mat=mat.reshape(1);

            mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
            mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
            mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
            mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = imLeft.cols;
            mnMinY = 0.0f;
            mnMaxY = imLeft.rows;
        }
    }

    void Frame::ComputeStereoMatches()
    {
        mvuRight = vector<float>(N,-1.0f);
        mvDepth = vector<float>(N,-1.0f);

        const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

        const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

        //Assign keypoints to row table
        vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

        for(int i=0; i<nRows; i++)
            vRowIndices[i].reserve(200);

        const int Nr = mvKeysRight.size();

        for(int iR=0; iR<Nr; iR++)
        {
            const cv::KeyPoint &kp = mvKeysRight[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
            const int maxr = ceil(kpY+r);
            const int minr = floor(kpY-r);

            for(int yi=minr;yi<=maxr;yi++)
                vRowIndices[yi].push_back(iR);
        }

        // Set limits for search
        const float minZ = mb;
        const float minD = 0;
        const float maxD = mbf/minZ;

        // For each left keypoint search a match in the right image
        vector<pair<int, int> > vDistIdx;
        vDistIdx.reserve(N);

        for(int iL=0; iL<N; iL++)
        {
            const cv::KeyPoint &kpL = mvKeys[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if(vCandidates.empty())
                continue;

            const float minU = uL-maxD;
            const float maxU = uL-minD;

            if(maxU<0)
                continue;

            int bestDist = ORBmatcher::TH_HIGH;
            size_t bestIdxR = 0;

            const cv::Mat &dL = mDescriptors.row(iL);

            // Compare descriptor to right keypoints
            for(size_t iC=0; iC<vCandidates.size(); iC++)
            {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = mvKeysRight[iR];

                if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                    continue;

                const float &uR = kpR.pt.x;

                if(uR>=minU && uR<=maxU)
                {
                    const cv::Mat &dR = mDescriptorsRight.row(iR);
                    const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                    if(dist<bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // Subpixel match by correlation
            if(bestDist<thOrbDist)
            {
                // coordinates in image pyramid at keypoint scale
                const float uR0 = mvKeysRight[bestIdxR].pt.x;
                const float scaleFactor = mvInvScaleFactors[kpL.octave];
                const float scaleduL = round(kpL.pt.x*scaleFactor);
                const float scaledvL = round(kpL.pt.y*scaleFactor);
                const float scaleduR0 = round(uR0*scaleFactor);

                // sliding window search
                const int w = 5;
                cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
                IL.convertTo(IL,CV_32F);
                IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = 5;
                vector<float> vDists;
                vDists.resize(2*L+1);

                const float iniu = scaleduR0+L-w;
                const float endu = scaleduR0+L+w+1;
                if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                    continue;

                for(int incR=-L; incR<=+L; incR++)
                {
                    cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                    IR.convertTo(IR,CV_32F);
                    IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                    float dist = cv::norm(IL,IR,cv::NORM_L1);
                    if(dist<bestDist)
                    {
                        bestDist =  dist;
                        bestincR = incR;
                    }

                    vDists[L+incR] = dist;
                }

                if(bestincR==-L || bestincR==L)
                    continue;

                // Sub-pixel match (Parabola fitting)
                const float dist1 = vDists[L+bestincR-1];
                const float dist2 = vDists[L+bestincR];
                const float dist3 = vDists[L+bestincR+1];

                const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

                if(deltaR<-1 || deltaR>1)
                    continue;

                // Re-scaled coordinate
                float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

                float disparity = (uL-bestuR);

                if(disparity>=minD && disparity<maxD)
                {
                    if(disparity<=0)
                    {
                        disparity=0.01;
                        bestuR = uL-0.01;
                    }
                    mvDepth[iL]=mbf/disparity;
                    mvuRight[iL] = bestuR;
                    vDistIdx.push_back(pair<int,int>(bestDist,iL));
                }
            }
        }

        sort(vDistIdx.begin(),vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size()/2].first;
        const float thDist = 1.5f*1.4f*median;

        for(int i=vDistIdx.size()-1;i>=0;i--)
        {
            if(vDistIdx[i].first<thDist)
                break;
            else
            {
                mvuRight[vDistIdx[i].second]=-1;
                mvDepth[vDistIdx[i].second]=-1;
            }
        }
    }


    void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
    {
        mvuRight = vector<float>(N,-1);
        mvDepth = vector<float>(N,-1);

        for(int i=0; i<N; i++)
        {
            const cv::KeyPoint &kp = mvKeys[i];
            const cv::KeyPoint &kpU = mvKeysUn[i];

            const float &v = kp.pt.y;
            const float &u = kp.pt.x;

            const float d = imDepth.at<float>(v,u);

            if(d>0)
            {
                mvDepth[i] = d;
                mvuRight[i] = kpU.pt.x-mbf/d;
            }
        }
    }

    cv::Mat Frame::UnprojectStereo(const int &i)
    {
        const float z = mvDepth[i];
        if(z>0)
        {
            const float u = mvKeysUn[i].pt.x;
            const float v = mvKeysUn[i].pt.y;
            const float x = (u-cx)*z*invfx;
            const float y = (v-cy)*z*invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
            return mRwc*x3Dc+mOw;
        }
        else
            return cv::Mat();
    }

    Vector6d Frame::obtain3DLine(const int &i)
    {
        Vector6d Lines3D=mvLines3D[i];
        cv::Mat Ac =(Mat_<float>(3,1) <<Lines3D(0),Lines3D(1),Lines3D(2));
        cv::Mat A= mRwc*Ac+mOw;
        cv::Mat Bc = (Mat_<float>(3,1) << Lines3D(3),Lines3D(4),Lines3D(5));
        cv::Mat B= mRwc*Bc+mOw;
        Lines3D<<A.at<float>(0,0),A.at<float>(1,0),A.at<float>(2,0),B.at<float>(0,0),B.at<float>(1,0),B.at<float>(2,0);
        //cout<<mvLines3D[i]<<endl;
        return Lines3D;
    }


} //namespace ORB_SLAM
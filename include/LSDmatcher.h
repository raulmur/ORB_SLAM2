//
// Created by lan on 17-12-26.
//

#ifndef ORB_SLAM2_LSDMATCHER_H
#define ORB_SLAM2_LSDMATCHER_H

//#include <line_descriptor/descriptor_custom.hpp>
//#include <line_descriptor_custom.hpp>

#include "MapLine.h"
#include "KeyFrame.h"
#include "Frame.h"

using namespace line_descriptor;

namespace ORB_SLAM2
{
    class LSDmatcher
    {
    public:
        LSDmatcher(float nnratio=0.6, bool checkOri=true);

        //获得匹配数量的同时，获得当前帧的mvMapLine
        int SearchByProjection(KeyFrame* pKF,Frame &currentF, vector<MapLine*> &vpMapLineMatches);

        // 通过投影线段，对上一帧的特征线进行跟踪
        int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame);

        // 通过投影，对Local MapLine进行跟踪
        int SearchByProjection(Frame &F, const std::vector<MapLine*> &vpMapLines, const float th=3);

        //计算匹配对
        int SearchByProjection(KeyFrame* pKF,KeyFrame *pKF2, vector<MapLine*> &vpMapLineMatches);
        int SerachForInitialize(Frame &InitialFrame, Frame &CurrentFrame, vector<pair<int,int>> &LineMatches);

        static int DescriptorDistance(const Mat &a, const Mat &b);

        int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, vector<pair<size_t, size_t>> &vMatchedPairs);

        // Project MapLines into KeyFrame and search for duplicated MapLines
        int Fuse(KeyFrame* pKF, const vector<MapLine *> &vpMapLines);

    public:

        static const int TH_LOW;
        static const int TH_HIGH;
        static const int HISTO_LENGTH;

    protected:
        float RadiusByViewingCos(const float &viewCos);

        float mfNNratio;
        bool mbCheckOrientation;
    };
}


#endif //ORB_SLAM2_LSDMATCHER_H

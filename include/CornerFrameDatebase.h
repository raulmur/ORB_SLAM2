//
// Created by yan on 12/7/18.
//

#ifndef ORB_SLAM2_CORNERFRAMEDATEBASE_H
#define ORB_SLAM2_CORNERFRAMEDATEBASE_H

#include "Frame.h"
#include <math.h>
#include "MapPoint.h"
#include <mutex>
namespace ORB_SLAM2{
class Frame;

class CornerFrameDatebase{

public:
    CornerFrameDatebase();
    //添加帧
    void addFrame(Frame pF);
    //释放一帧
    void eraseFrame();

    //清除
    void clear();

    //计算角度
    double computeCornerAngle();
    //获得当前帧数
    int getSizeofCornerFrames();
    vector<Frame> getCornerFrame();
    //五张图的mappoint
    vector<MapPoint *> getCornerMappoint();

    int cornerObservation();
    void prepareCornerInformation();



protected:
    std::vector<Frame> cornerFrameDatabase;
    std::list<Frame> mqcornerFrameDatabase;

    vector<MapPoint *> vpCornerMP;
    // Mutex
    std::mutex mMutex;
    Frame * tmpFrame;

};

}


#endif //ORB_SLAM2_CORNERFRAMEDATEBASE_H

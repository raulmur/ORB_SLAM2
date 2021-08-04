//
// Created by yan on 12/7/18.
//

#include "CornerFrameDatebase.h"

namespace ORB_SLAM2{

CornerFrameDatebase::CornerFrameDatebase()
{


}

void CornerFrameDatebase::addFrame(Frame  pF){
    unique_lock<mutex> lock(mMutex);
    //cornerFrameDatabase.push_back(pF);
    mqcornerFrameDatabase.push_back(pF);
    cornerFrameDatabase.push_back(pF);

}

void CornerFrameDatebase::eraseFrame()
{
    if(!mqcornerFrameDatabase.empty())
        mqcornerFrameDatabase.pop_front();
    //cornerFrameDatabase.erase(pF);
}

double CornerFrameDatebase::computeCornerAngle()
{
        vector<cv::Point3d> vCornertcw;
        double angleOfCorner=0;
        cv::Point3d leftVec,rightVec;
        list<Frame>::iterator i;
        for( i=mqcornerFrameDatabase.begin();i!=mqcornerFrameDatabase.end();i++)
        {

            Frame tmpFrame = *i;
            vCornertcw.push_back(cv::Point3d(tmpFrame.mTcw.at<float>(0, 3), tmpFrame.mTcw.at<float>(1, 3),
                                             tmpFrame.mTcw.at<float>(2, 3)));
        }


        //获得梯度,让他们相减
        for(size_t i=1;i<vCornertcw.size();i++)
        {
            leftVec=vCornertcw[i]-vCornertcw[0];
            for(size_t j=i+1;j<vCornertcw.size();j++)
            {
                rightVec=vCornertcw[j]-vCornertcw[0];
                    double costemp1=(leftVec.x*rightVec.x+leftVec.y*rightVec.y+leftVec.z*rightVec.z);
                    double costemp2=norm(leftVec)*norm(rightVec);
                    double angle=acos(costemp1/costemp2) * 180.0 / 3.14;
		            if(angle>angleOfCorner) angleOfCorner=angle;
                    //cout<< angleOfCorner<<endl;
            }
        }
        return angleOfCorner;

    }

vector<MapPoint *> CornerFrameDatebase::getCornerMappoint()
{

}

vector<Frame> CornerFrameDatebase::getCornerFrame(){

    vector<Frame> tmpFrame;
    int count=mqcornerFrameDatabase.size();
    cout<<count<<",count"<<endl;
    for(int i=0;i<count;i++){
        Frame tmp=mqcornerFrameDatabase.front();
        cout<<"ssize:"<<mqcornerFrameDatabase.size()<<tmp.mnId<<endl;
        tmpFrame.push_back(tmp);
        mqcornerFrameDatabase.pop_front();
    }

    return tmpFrame;
}

int CornerFrameDatebase::getSizeofCornerFrames()
{
        return mqcornerFrameDatabase.size();
    }

}

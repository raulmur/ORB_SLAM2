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

#ifndef MAP_H
#define MAP_H

#include <mutex>
#include <set>

#include "MapBase.h"

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

class Map : public MapBase
{
public:
    Map();

    virtual void AddKeyFrame(KeyFrame* pKF) override;
    virtual void AddMapPoint(MapPoint* pMP) override;
    virtual void EraseMapPoint(MapPoint* pMP) override;
    virtual void EraseKeyFrame(KeyFrame* pKF) override;
    virtual void SetReferenceMapPoints(
        const std::vector<MapPoint*> &vpMPs) override;

    virtual std::vector<KeyFrame*> GetAllKeyFrames() override;
    virtual std::vector<MapPoint*> GetAllMapPoints() override;
    virtual std::vector<MapPoint*> GetReferenceMapPoints() override;

    virtual long unsigned int MapPointsInMap() override;
    virtual long unsigned  KeyFramesInMap() override;

    virtual long unsigned int GetMaxKFid() override;

    virtual void clear() override;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H

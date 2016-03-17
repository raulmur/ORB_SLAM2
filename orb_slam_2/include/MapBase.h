#pragma once

#include <mutex>
#include <vector>

namespace ORB_SLAM2
{

class KeyFrame;
class MapPoint;

class MapBase
{
 public:
  virtual ~MapBase() = default;
  virtual void AddKeyFrame(KeyFrame* pKF) = 0;
  virtual void AddMapPoint(MapPoint* pMP) = 0;
  virtual void EraseMapPoint(MapPoint* pMP) = 0;
  virtual void EraseKeyFrame(KeyFrame* pKF) = 0;
  virtual void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs) = 0;

  virtual std::vector<KeyFrame*> GetAllKeyFrames() = 0;
  virtual std::vector<MapPoint*> GetAllMapPoints() = 0;
  virtual std::vector<MapPoint*> GetReferenceMapPoints() = 0;

  virtual long unsigned int MapPointsInMap() = 0;
  virtual long unsigned  KeyFramesInMap() = 0;

  virtual long unsigned int GetMaxKFid() = 0;

  virtual void clear() = 0;

  std::vector<KeyFrame*> mvpKeyFrameOrigins;

  std::mutex mMutexMapUpdate;

  // This avoid that two points are created simultaneously in separate threads (id conflict)
  std::mutex mMutexPointCreation;
};

}  // namespace ORB_SLAM2

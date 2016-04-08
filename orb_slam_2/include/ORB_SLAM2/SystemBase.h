#ifndef ORB_SLAM2_SYSTEMBASE_H_
#define ORB_SLAM2_SYSTEMBASE_H_

namespace ORB_SLAM2 {

class SystemBase
{
 public:
  virtual ~SystemBase() = default;

  virtual void Reset() = 0;
};

}  // namespace ORB_SLAM2

#endif  // ORB_SLAM2_SYSTEMBASE_H_

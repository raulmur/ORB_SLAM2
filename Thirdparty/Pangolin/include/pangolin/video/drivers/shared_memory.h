#pragma once

#include <pangolin/pangolin.h>
#include <pangolin/compat/memory.h>
#include <pangolin/video/video.h>
#include <pangolin/utils/posix/condition_variable.h>
#include <pangolin/utils/posix/shared_memory_buffer.h>

#include <vector>

namespace pangolin
{

class SharedMemoryVideo : public VideoInterface
{
public:
  SharedMemoryVideo(size_t w, size_t h, std::string pix_fmt,
    const boostd::shared_ptr<SharedMemoryBufferInterface>& shared_memory,
    const boostd::shared_ptr<ConditionVariableInterface>& buffer_full);
  ~SharedMemoryVideo();

  size_t SizeBytes() const;
  const std::vector<StreamInfo>& Streams() const;
  void Start();
  void Stop();
  bool GrabNext(unsigned char *image, bool wait);
  bool GrabNewest(unsigned char *image, bool wait);

private:
  VideoPixelFormat _fmt;
  size_t _frame_size;
  std::vector<StreamInfo> _streams;
  boostd::shared_ptr<SharedMemoryBufferInterface> _shared_memory;
  boostd::shared_ptr<ConditionVariableInterface> _buffer_full;
};

}

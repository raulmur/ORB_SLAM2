#pragma once

#include <pangolin/compat/memory.h>

#include <string>

namespace pangolin
{

  class SemaphoreInterface
  {
  public:

    virtual ~SemaphoreInterface() {
    }

    virtual bool tryAcquire() = 0;
    virtual void acquire() = 0;
    virtual void release() = 0;
  };

  boostd::shared_ptr<SemaphoreInterface> create_named_semaphore(const std::string& name,
    unsigned int value);
  boostd::shared_ptr<SemaphoreInterface> open_named_semaphore(const std::string& name);

}

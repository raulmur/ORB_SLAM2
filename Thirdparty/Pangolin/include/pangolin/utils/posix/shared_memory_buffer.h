#pragma once

#include <pangolin/compat/memory.h>

#include <string>

namespace pangolin
{
  class SharedMemoryBufferInterface
  {
  public:
    virtual ~SharedMemoryBufferInterface() {
    }
    virtual bool tryLock() = 0;
    virtual void lock() = 0;
    virtual void unlock() = 0;
    virtual unsigned char *ptr() = 0;
    virtual std::string name() = 0;
  };

  boostd::shared_ptr<SharedMemoryBufferInterface> create_named_shared_memory_buffer(const
    std::string& name, size_t size);
  boostd::shared_ptr<SharedMemoryBufferInterface> open_named_shared_memory_buffer(const
    std::string& name, bool readwrite);
}

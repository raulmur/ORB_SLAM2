#pragma once

#include <pangolin/compat/memory.h>
#include <pangolin/utils/timer.h>

namespace pangolin
{

class ConditionVariableInterface
{
public:
  virtual ~ConditionVariableInterface()
  {
  }

  virtual void wait() = 0;
  virtual bool wait(basetime t) = 0;
  virtual void signal() = 0;
  virtual void broadcast() = 0;
};

boostd::shared_ptr<ConditionVariableInterface> create_named_condition_variable(const
  std::string& name);
boostd::shared_ptr<ConditionVariableInterface> open_named_condition_variable(const
  std::string& name);
}

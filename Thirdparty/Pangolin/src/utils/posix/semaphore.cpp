#include <pangolin/utils/posix/semaphore.h>

#include <fcntl.h>
#include <semaphore.h>
#include <sys/stat.h>

#include <string>

using namespace std;

namespace pangolin
{

// TODO(shaheen) register a signal handler for SIGTERM and unlink shared
// semaphores.
class PosixSemaphore : public SemaphoreInterface
{
public:
  PosixSemaphore(sem_t *semaphore, bool ownership, const string& name) :
    _semaphore(semaphore),
    _ownership(ownership),
    _name(name)
  {
  }

  ~PosixSemaphore()
  {
    if (_ownership) {
      sem_unlink(_name.c_str());
    } else {
      sem_close(_semaphore);
    }
  }

  bool tryAcquire()
  {
    int err = sem_trywait(_semaphore);
    return err == 0;
  }

  void acquire()
  {
    sem_wait(_semaphore);
  }

  void release()
  {
    sem_post(_semaphore);
  }

private:
  sem_t *_semaphore;
  bool _ownership;
  string _name;
};

boostd::shared_ptr<SemaphoreInterface> create_named_semaphore(const string& name, unsigned int value)
{
  boostd::shared_ptr<SemaphoreInterface> ptr;
  sem_t *semaphore = sem_open(name.c_str(), O_CREAT | O_EXCL,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, value);
  if (NULL == semaphore) {
    return ptr;
  }

  ptr.reset(new PosixSemaphore(semaphore, true, name));
  return ptr;
}

boostd::shared_ptr<SemaphoreInterface> open_named_semaphore(const string& name)
{
  boostd::shared_ptr<SemaphoreInterface> ptr;
  sem_t *semaphore = sem_open(name.c_str(), 0);

  if (NULL == semaphore) {
    return ptr;
  }

  ptr.reset(new PosixSemaphore(semaphore, false, name));
  return ptr;
}

}

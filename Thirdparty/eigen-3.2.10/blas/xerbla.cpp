
#include <iostream>

#if (defined __GNUC__) && (!defined __MINGW32__) && (!defined __CYGWIN__)
#define EIGEN_WEAK_LINKING __attribute__ ((weak))
#else
#define EIGEN_WEAK_LINKING
#endif

#ifdef __cplusplus
extern "C"
{
#endif

EIGEN_WEAK_LINKING int xerbla_(const char * msg, int *info, int)
{
  std::cerr << "Eigen BLAS ERROR #" << *info << ": " << msg << "\n";
  return 0;
}

#ifdef __cplusplus
}
#endif

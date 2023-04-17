#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef WIN32
  #ifndef DBoW2_EXPORTS
	#define DBoW2_EXPORTS
  #endif
  #ifdef DBoW2_EXPORTS
    #define EXPORT //__declspec(dllexport)
  #else
    #define EXPORT //__declspec(dllimport) 
  #endif
#else
  #define EXPORT 
#endif

#endif // CONFIG_H

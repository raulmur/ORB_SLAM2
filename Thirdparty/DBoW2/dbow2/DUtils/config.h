#ifndef CONFIG_H
#define CONFIG_H

//#ifndef DBoW2_EXPORTS
//	#define DBoW2_EXPORTS
//#endif

#ifdef WIN32
#ifndef DBoW2_EXPORTS
	#define DBoW2_EXPORTS
#endif
#ifdef DBoW2_EXPORTS
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __declspec(dllimport) 
#endif
#else
#define EXPORT 
//#define REGISTER_TYPE(name, classname) \
//		extern "C" void EXPORT ##classname(void) {} 
#endif


//#ifdef DBoW2_EXPORTS
//	#define EXPORT __declspec(dllexport)
//#else
//	#define EXPORT __declspec(dllimport) 
//#endif
//
//#endif 

#endif // CONFIG_H


//#define REGISTER_TYPE(name, classname) \
//		extern "C" void EXPORT ##classname(void) {} \
#pragma once

#ifdef AUTODRIVECORE_EXPORTS
#define AUTODRIVECORE_EXPORT extern "C" __declspec(dllexport) 
#else
#define AUTODRIVECORE_EXPORT extern "C" __declspec(dllimport)
#endif

#define DEGREE_TO_RADIAN(deg) deg * 0.0174532925199432957692369076849
#define RADIAN_TO_DEGREE(rad) rad * 57.295779513082320876798154814105



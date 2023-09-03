#pragma once

#ifdef AUTODRIVECORE_EXPORTS
#define AUTODRIVECORE_EXPORT extern "C" __declspec(dllexport) 
#else
#define AUTODRIVECORE_EXPORT extern "C" __declspec(dllimport)
#endif

#define PCL_NEW(TYPE, VAR)	TYPE::Ptr VAR(new TYPE)

#define DEGREE_TO_RADIAN(deg) deg * 0.0174532925199432957692369076849
#define RADIAN_TO_DEGREE(rad) rad * 57.295779513082320876798154814105


#define AUTODRIVECORE_DEF(TYPE, VAR)	private: TYPE m_##VAR;
#define AUTODRIVECORE_GET(TYPE, VAR)	public: TYPE Get##VAR() { return m_##VAR; }
#define AUTODRIVECORE_SET(TYPE, VAR)	public: void Set##VAR(TYPE _##VAR) { m_##VAR = _##VAR; }

#define AUTODRIVECORE_CLASS_GET_SET(TYPE, VAR)	\
AUTODRIVECORE_DEF(TYPE, VAR)					\
AUTODRIVECORE_GET(TYPE, VAR)					\
AUTODRIVECORE_SET(TYPE, VAR)

#define AUTODRIVECORE_CLASS_GET(TYPE, VAR)		\
AUTODRIVECORE_DEF(TYPE, VAR)					\
AUTODRIVECORE_GET(TYPE, VAR)

#define AUTODRIVECORE_CLASS_SET(TYPE, VAR)		\
AUTODRIVECORE_DEF(TYPE, VAR)					\
AUTODRIVECORE_SET(TYPE, VAR)

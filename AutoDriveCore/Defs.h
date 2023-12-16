#pragma once

#ifdef AUTODRIVECORE_EXPORTS
#define AUTODRIVECORE_EXPORT extern "C" __declspec(dllexport) 
#else
#define AUTODRIVECORE_EXPORT extern "C" __declspec(dllimport)
#endif



#define CAMERA_IMAGE_WIDTH						1280
#define CAMERA_IMAGE_HEIGHT						960
#define CAMERA_WIDTH_MPP_IN_ONE_MM_DISTANCE		0.00078125
#define CAMERA_HEIGHT_MPP_IN_ONE_MM_DISTANCE	0.00078125
#define CAMERA_ROLL_DEGREE						-1.5
#define CAMERA_EDGE_BAD_PIXEL_MARGIN			4
#define CAMERA_TO_YAW_AXIS_Y_OFFSET				30
#define YAW_AXIS_TO_CENTER_Y_OFFSET				130
#define LIDAR_DEGREE_OFFSET						181.5
#define LIDAR_HEIGHT_OFFSET						70
#define IMAGE_TO_POINT_SIZE_RATE				0.125
#define IMAGE_POINT_DEFAULT_DISTANCE			1000.0



#define DEGREE_TO_RADIAN(deg) deg * 0.0174532925199432957692369076849
#define RADIAN_TO_DEGREE(rad) rad * 57.295779513082320876798154814105

#define PCL_NEW(TYPE, VAR)	TYPE::Ptr VAR(new TYPE)

#define DEBUG_PRINT(STR) OutputDebugStringA(STR)



#define AUTODRIVECORE_MODULE_START		namespace AutoDriveCode { namespace Modules {
#define AUTODRIVECORE_MODULE_END		}}

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

#define AUTODRIVECORE_DEF(TYPE, VAR)	protected: TYPE m_##VAR;
#define AUTODRIVECORE_GET(TYPE, VAR)	public: TYPE Get##VAR() { return m_##VAR; }
#define AUTODRIVECORE_SET(TYPE, VAR)	public: void Set##VAR(TYPE _##VAR) { m_##VAR = _##VAR; }



#pragma once
#include "TypeToTypeFilter.h"


AUTODRIVECORE_MODULE_START

class ImageToPointCloudFilterType
{
	AUTODRIVECORE_CLASS_GET_SET(cv::Mat, InputImage);
	AUTODRIVECORE_CLASS_GET_SET(PTLNCPtr, OutputPointCloud);

	AUTODRIVECORE_CLASS_GET_SET(int, Margin);
	AUTODRIVECORE_CLASS_GET_SET(double, Scale);
	AUTODRIVECORE_CLASS_GET_SET(double, DefaultDistance);
	AUTODRIVECORE_CLASS_GET_SET(double, OffsetDistance);
	AUTODRIVECORE_CLASS_GET_SET(double, WidthMPPInOneMmDistance);
	AUTODRIVECORE_CLASS_GET_SET(double, HeightMPPInOneMmDistance);
	AUTODRIVECORE_CLASS_GET_SET(double, CenterX);
	AUTODRIVECORE_CLASS_GET_SET(double, CenterY);
	AUTODRIVECORE_CLASS_GET_SET(double, CenterZ);
	AUTODRIVECORE_CLASS_GET_SET(double, RollDegree);
	AUTODRIVECORE_CLASS_GET_SET(double, PitchDegree);
	AUTODRIVECORE_CLASS_GET_SET(double, YawDegree);
	AUTODRIVECORE_CLASS_GET_SET(Eigen::Matrix4f, MoveMatrix);

protected:
	PTNCPtr m_LastCurvedOffset = NULL;
	int m_LastWidth = 0;
	int m_LastHeight = 0;
	double m_LastScale = -1.0;
	double m_LastDefaultDistance = -1.0;
	double m_LastOffsetDistance = -1.0;
	double m_LastWidthMPPInOneMmDistance = -1.0;
	double m_LastHeightMPPInOneMmDistance = -1.0;

	void initCurvedOffset(int w, int h);

public:
	ImageToPointCloudFilterType();
	virtual void Update();
};

AUTODRIVECORE_MODULE_END

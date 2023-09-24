#pragma once
#include "TypeToTypeFilter.h"


AUTODRIVECORE_MODULE_START

class RotateCropImageFilterType
	: public ImageToImageFilterType
{
	AUTODRIVECORE_CLASS_GET_SET(int, Margin);
	AUTODRIVECORE_CLASS_GET_SET(double, RotateDegree);
	AUTODRIVECORE_CLASS_GET_SET(double, GaussianSigma);
	AUTODRIVECORE_CLASS_GET_SET(bool, IsNormalized);

public:
	RotateCropImageFilterType();
	virtual void Update();
};

AUTODRIVECORE_MODULE_END

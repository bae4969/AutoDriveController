#pragma once
#include "TypeToTypeFilter.h"


AUTODRIVECORE_MODULE_START

class ImageRegistrationFilterType
	: public PointCloudToPointCloudFilterType<PTLNCPtr, PTLNCPtr>
{

};

AUTODRIVECORE_MODULE_END

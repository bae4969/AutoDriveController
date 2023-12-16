#pragma once
#include "TypeToTypeFilter.h"


AUTODRIVECORE_MODULE_START

class ImageRegistrationFilterType
	: public PointCloudToPointCloudFilterType<PTLNCType, PTLNCType>
{

};

AUTODRIVECORE_MODULE_END

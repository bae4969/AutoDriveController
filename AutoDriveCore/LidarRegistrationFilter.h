#pragma once
#include "TypeToTypeFilter.h"


AUTODRIVECORE_MODULE_START

class LidarRegistrationFilterType
	: public PointCloudToPointCloudFilterType<PTNCType, PTNCType>
{

};

AUTODRIVECORE_MODULE_END
#pragma once
#include "TypeToTypeFilter.h"


AUTODRIVECORE_MODULE_START

template<typename InputType, typename OutputType>
class ICPPointCloudFilterType
	: public PointCloudToPointCloudFilterType<InputType, OutputType>
{

};

AUTODRIVECORE_MODULE_END

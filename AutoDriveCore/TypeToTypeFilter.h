#pragma once
#include <opencv2/core.hpp>
#include "Defs.h"
#include "Types.h"


AUTODRIVECORE_MODULE_START

class ImageToImageFilterType
{
	AUTODRIVECORE_CLASS_GET_SET(bool, Inplace);
	AUTODRIVECORE_CLASS_GET_SET(cv::Mat, InputImage);
	AUTODRIVECORE_CLASS_GET(cv::Mat, OutputImage);

public:
	virtual void Update() {}
};

template<typename InputPcType, typename OutputPcType>
class PointCloudToPointCloudFilterType
{
	AUTODRIVECORE_CLASS_GET_SET(bool, Inplace);
	AUTODRIVECORE_CLASS_GET_SET(InputPcType::Ptr, InputPointCloud);
	AUTODRIVECORE_CLASS_GET(OutputPcType::Ptr, OutputPointCloud);

public:
	virtual void Update() {}
};

AUTODRIVECORE_MODULE_END


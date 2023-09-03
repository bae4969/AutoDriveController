#pragma once
#include "Types.h"

namespace AutoDriveCode {
	namespace ImageFilter {
		cv::Mat DrawStateInfoFilter(cv::Mat& src, MachineStateType& stateInfo);
		cv::Mat CalibrationCameraFilter(cv::Mat& src, CameraCaliDataType& caliData);
		PTLCPtr ConvertImageToPointCloud(cv::Mat& src, CameraCaliDataType& caliData, MachineStateType& stateInfo);
	}
}


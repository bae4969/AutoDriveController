#pragma once
#include "Types.h"

namespace AutoDriveCode {
	namespace ImageFilter {
		cv::Mat DrawStateInfoFilter(cv::Mat src, MachineStateType& stateInfo);
		cv::Mat CalibrationCameraFilter(cv::Mat src, int margin, double rotDeg);

		////////////////////////////////////////////////////////////////////////////////

		PTLCPtr ConvertImageToPointCloud(cv::Mat src, int margin, cv::Size resizedSize, PTCPtr caliOffset, float pitchDeg, float yawDeg);
	}
}


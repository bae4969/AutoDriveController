#pragma once
#include "Types.h"

namespace AutoDriveCode {
	namespace ImageFilter {
		cv::Mat ApplyStateInfo(cv::Mat& src, StateType& stateInfo);
		cv::Mat ApplyAdaptiveBrightness(cv::Mat& src);
	}
}

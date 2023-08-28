#pragma once
#include "Defs.h"
#include <opencv2/core.hpp>

namespace AutoDriveCode {
	struct ImageData {
		int W;
		int H;
		int Ch;
		int Step;
		void* Data;

		ImageData();
		ImageData(cv::Mat mat);
		cv::Mat ToMat();
		void Update(cv::Mat mat);
		void Release();
	};

}



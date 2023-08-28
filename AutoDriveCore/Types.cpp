#include "pch.h"
#include "Types.h"

namespace AutoDriveCode {
	using namespace std;
	using namespace cv;

	ImageData::ImageData() {
		W = 0;
		H = 0;
		Ch = 0;
		Step = 0;
		Data = NULL;
	}
	ImageData::ImageData(Mat mat) {
		W = mat.cols;
		H = mat.rows;
		Ch = mat.elemSize();
		Step = mat.elemSize1();

		size_t totMemSize = W * H * Ch * Step;
		Data = new char[totMemSize];
		memcpy(Data, mat.data, totMemSize);
	}
	Mat ImageData::ToMat() {
		int depth;
		switch (Step) {
		case 1: depth = CV_8U; break;
		case 2: depth = CV_16U; break;
		case 4: depth = CV_32F; break;
		case 8: depth = CV_64F; break;
		default: return Mat();
		}

		int type = CV_MAKETYPE(depth, Ch);
		return Mat(H, W, type, Data);
	}
	void ImageData::Update(Mat mat) {
		if (W == mat.cols &&
			H == mat.rows &&
			Ch == mat.elemSize() &&
			Step == mat.elemSize1())
		{
			size_t totMemSize = W * H * Ch * Step;
			memcpy(Data, mat.data, totMemSize);
		}
		else {
			W = mat.cols;
			H = mat.rows;
			Ch = mat.elemSize();
			Step = mat.elemSize1();

			size_t totMemSize = W * H * Ch * Step;
			if (Data)
				delete[] Data;
			Data = new char[totMemSize];
			memcpy(Data, mat.data, totMemSize);
		}
	}
	void ImageData::Release() {
		if (Data) {
			delete[] Data;
			Data = NULL;
		}
	}
}



#include "pch.h"
#include "AutoDriveCore.h"

namespace AutoDriveCode {
	using namespace std;
	using namespace cv;

	int NormalizeRGB(int w, int h, void* data) {
		Mat temp = Mat::zeros(h, w, CV_8UC3);
		memcpy(temp.data, data, w * h * 3);

		double minv = DBL_MAX;
		double maxv = DBL_MIN;
		Scalar meanVal;
		Scalar stdDev;
		meanStdDev(temp, meanVal, stdDev);
		for (int i = 0; i < 3; i++) {
			double t_minv = meanVal[i] - stdDev[i];
			double t_maxv = meanVal[i] + stdDev[i];
			if (minv > t_minv)
				minv = t_minv;
			if (maxv < t_maxv)
				maxv = t_maxv;
		}
		temp.setTo(minv, temp < minv);
		temp.setTo(maxv, temp > maxv);
		normalize(temp, temp, 0, UCHAR_MAX, NORM_MINMAX);



		//vector<Mat> rgb;
		//split(temp, rgb);
		//for (int i = 0; i < rgb.size(); i++) {

		//	int minv = meanVal[0] - stdDev[0];
		//	int maxv = meanVal[0] + stdDev[0];
		//}

		//merge(rgb, temp);
		memcpy(data, temp.data, w * h * 3);

		return 1;
	}

	int TEST(int w, int h, void* data) {
		cv::Mat temp = cv::Mat::zeros(h, w, CV_8UC3);

		memcpy(temp.data, data, w * h * 3);

		cv::imwrite("test.png", temp);

		return 1;
	}
}

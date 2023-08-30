#include "pch.h"
#include "ImageFilter.h"

namespace AutoDriveCode {
	namespace ImageFilter {
		using namespace std;
		using namespace cv;

		Mat ApplyFilter(Mat& src) {
			Mat dst = src.clone();
			Size size = src.size();

			double minv = DBL_MAX;
			double maxv = DBL_MIN;
			Scalar meanVal;
			Scalar stdDev;
			meanStdDev(dst, meanVal, stdDev);
			for (int i = 0; i < 3; i++) {
				double t_minv = meanVal[i] - 2.0 * stdDev[i];
				double t_maxv = meanVal[i] + 2.0 * stdDev[i];
				if (minv > t_minv)
					minv = t_minv;
				if (maxv < t_maxv)
					maxv = t_maxv;
			}
			dst.setTo(minv, dst < minv);
			dst.setTo(maxv, dst > maxv);
			normalize(dst, dst, 0, UCHAR_MAX, NORM_MINMAX);

			Mat rotMat = getRotationMatrix2D(Point2f(size.width * 0.5, size.height * 0.5), -2.0, 1.0);
			warpAffine(dst, dst, rotMat, Size());

			return dst;
		}
	}
}

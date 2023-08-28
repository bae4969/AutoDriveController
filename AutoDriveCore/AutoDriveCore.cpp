#include "pch.h"
#include "AutoDriveCore.h"

namespace AutoDriveCode {
	using namespace std;
	using namespace cv;

	ImageData g_OutData;

	ImageData ApplyImageFilter(ImageData in_data) {
		Mat img = in_data.ToMat().clone();
		in_data.Release();
		Size size = img.size();

		double minv = DBL_MAX;
		double maxv = DBL_MIN;
		Scalar meanVal;
		Scalar stdDev;
		meanStdDev(img, meanVal, stdDev);
		for (int i = 0; i < 3; i++) {
			double t_minv = meanVal[i] - 2.0 * stdDev[i];
			double t_maxv = meanVal[i] + 2.0 * stdDev[i];
			if (minv > t_minv)
				minv = t_minv;
			if (maxv < t_maxv)
				maxv = t_maxv;
		}
		img.setTo(minv, img < minv);
		img.setTo(maxv, img > maxv);
		normalize(img, img, 0, UCHAR_MAX, NORM_MINMAX);

		Mat rotMat = getRotationMatrix2D(Point2f(size.width * 0.5, size.height * 0.5), -2.0, 1.0);
		warpAffine(img, img, rotMat, Size());

		g_OutData.Update(img);

		return g_OutData;
	}
}

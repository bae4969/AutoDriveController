#include "pch.h"
#include "RotateCropImageFilter.h"

AUTODRIVECORE_MODULE_START

using namespace std;
using namespace cv;

RotateCropImageFilterType::RotateCropImageFilterType() {
	m_Margin = 0;
	m_RotateDegree = 0.0;
	m_GaussianSigma = 0.0;
	m_IsNormalized = false;
}
void RotateCropImageFilterType::Update() {
	Mat dst = m_Inplace ? m_InputImage : m_InputImage.clone();
	Size srcSize = dst.size();

	// 외각에 생기는 bad pixel 자르기
	if (m_Margin > 0) {
		Rect edgeCut(m_Margin, m_Margin, dst.cols - m_Margin * 2, dst.rows - m_Margin * 2);
		dst = dst(edgeCut).clone();
		srcSize = dst.size();
	}

	// 회전 및 빈공간 자르기
	if (m_RotateDegree != 0.0) {
		Point2f rotCenter(srcSize.width * 0.5, srcSize.height * 0.5);
		Mat rotMat = getRotationMatrix2D(rotCenter, m_RotateDegree, 1.0);
		warpAffine(dst, dst, rotMat, Size(), INTER_LINEAR);

		Point2d rb(srcSize.width, srcSize.height);
		Point2d rotOrigin;
		Size2d rotSize;
		rotOrigin.x = abs(srcSize.width - (rotMat.at<double>(0, 0) * srcSize.width + rotMat.at<double>(0, 2)));
		rotOrigin.y = abs(srcSize.height - (rotMat.at<double>(1, 1) * srcSize.height + rotMat.at<double>(1, 2)));
		rotSize.width = srcSize.width - rotOrigin.x * 2.0;
		rotSize.height = srcSize.height - rotOrigin.y * 2.0;

		Rect rotCut(rotOrigin, rotSize);
		dst = dst(rotCut).clone();
		srcSize = dst.size();
	}

	// 단순 블러
	if (m_GaussianSigma > 0.0) {
		GaussianBlur(dst, dst, Size(), m_GaussianSigma);
	}

	if (m_IsNormalized) {
		normalize(dst, dst, 0, UCHAR_MAX, NORM_MINMAX);
	}

	m_OutputImage = dst;
}

AUTODRIVECORE_MODULE_END

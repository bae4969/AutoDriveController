#include "pch.h"
#include "ImageToPointCloudFilter.h"

AUTODRIVECORE_MODULE_START

using namespace std;
using namespace cv;
using namespace pcl;

ImageToPointCloudFilterType::ImageToPointCloudFilterType() {
	m_Margin = 0;
	m_Scale = 1.0;
	m_DefaultDistance = 1000.0;
	m_OffsetDistance = 0.0;
	m_WidthMPPInOneMmDistance = 1.0;
	m_HeightMPPInOneMmDistance = 1.0;
	m_CenterX = 0.0;
	m_CenterY = 0.0;
	m_CenterZ = 0.0;
	m_RollDegree = 0.0;
	m_PitchDegree = 0.0;
	m_YawDegree = 0.0;
	m_MoveMatrix .setIdentity();
}
void ImageToPointCloudFilterType::initCurvedOffset(int w, int h) {
	if (m_LastCurvedOffset &&
		m_LastWidth == w &&
		m_LastHeight == h &&
		m_LastScale == m_Scale &&
		m_LastDefaultDistance == m_DefaultDistance &&
		m_LastOffsetDistance == m_OffsetDistance &&
		m_LastWidthMPPInOneMmDistance == m_WidthMPPInOneMmDistance &&
		m_LastHeightMPPInOneMmDistance == m_HeightMPPInOneMmDistance)
		return;


	float halfWidth = w * 0.5;
	float halfHeight = h * 0.5;
	double xScale = m_WidthMPPInOneMmDistance * m_DefaultDistance / m_Scale;
	double zScale = m_HeightMPPInOneMmDistance * m_DefaultDistance / m_Scale;

	PCL_NEW(PTCType, src);
	src->resize(w * h);
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			size_t idx = y * w + x;
			PTType& pt = src->at(idx);
			pt.x = xScale * (x - halfWidth);
			pt.y = m_DefaultDistance;
			pt.z = zScale * (halfHeight - y);
			float scalrValue_inv = 1.f / sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
			float multiValue = m_DefaultDistance * scalrValue_inv;

			pt.x = pt.x * scalrValue_inv * m_DefaultDistance;
			pt.y = pt.y * scalrValue_inv * m_DefaultDistance;
			pt.z = pt.z * scalrValue_inv * m_DefaultDistance;
		}
	}

	PCL_NEW(PTCType, dst);
	Eigen::Matrix4f translationMat = Eigen::Matrix4f::Identity();
	translationMat.col(3)[1] = m_OffsetDistance;
	pcl::transformPointCloud(*src, *dst, translationMat);

	m_LastCurvedOffset = dst;
	m_LastWidth = w;
	m_LastHeight = h;
	m_LastScale = m_Scale;
	m_LastDefaultDistance = m_DefaultDistance;
	m_LastOffsetDistance = m_OffsetDistance;
	m_LastWidthMPPInOneMmDistance = m_WidthMPPInOneMmDistance;
	m_LastHeightMPPInOneMmDistance = m_HeightMPPInOneMmDistance;
}
void ImageToPointCloudFilterType::Update() {
	Mat cutImg;
	if (m_Margin > 0) {
		Rect edgeCut(m_Margin, m_Margin, m_InputImage.cols - m_Margin * 2, m_InputImage.rows - m_Margin * 2);
		cutImg = m_InputImage(edgeCut);
	}
	else {
		cutImg = m_InputImage;
	}

	Mat resizedImg;
	if (m_Scale != 1.0) {
		Size rescaleSize = cutImg.size();
		rescaleSize.width *= m_Scale;
		rescaleSize.height *= m_Scale;
		resize(cutImg, resizedImg, rescaleSize);
	}
	else {
		resizedImg = cutImg.clone();
	}

	Size imgSize = resizedImg.size();
	initCurvedOffset(imgSize.width, imgSize.height);

	PCL_NEW(PTLNCType, srcPc);
	srcPc->resize(imgSize.area());
	copyPointCloud<PTType, PTLNType>(*m_LastCurvedOffset, *srcPc);
	for (int idx = 0; idx < imgSize.area(); idx++) {
		PTLNType& src_pt = srcPc->at(idx);
		Vec3b& pxl = resizedImg.at<Vec3b>(idx);
		src_pt.normal_x = pxl[0] / 255.f;
		src_pt.normal_y = pxl[1] / 255.f;
		src_pt.normal_z = pxl[2] / 255.f;
		src_pt.curvature = 0.0;
		src_pt.b = pxl[0];
		src_pt.g = pxl[1];
		src_pt.r = pxl[2];
		src_pt.a = 255;
	}

	Eigen::AngleAxisd roll(DEGREE_TO_RADIAN(m_RollDegree), -Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitch(DEGREE_TO_RADIAN(m_PitchDegree), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yaw(DEGREE_TO_RADIAN(m_YawDegree), -Eigen::Vector3d::UnitZ());
	Eigen::Matrix3d t_rotMat = (yaw * pitch * roll).matrix();
	Eigen::Matrix4f rotationMat = Eigen::Matrix4f::Identity();
	for (int i = 0; i < 3; i++)for (int j = 0; j < 3; j++)
		rotationMat.col(i)[j] = t_rotMat.col(i)[j];

	Eigen::Matrix4f translationMat = Eigen::Matrix4f::Identity();
	translationMat.col(3)[0] = m_CenterX;
	translationMat.col(3)[1] = m_CenterY;
	translationMat.col(3)[2] = m_CenterZ;


	PCL_NEW(PTLNCType, dst);
	Eigen::Matrix4f totMat = m_MoveMatrix * translationMat * rotationMat;
	pcl::transformPointCloud(*srcPc, *dst, totMat);

	m_OutputPointCloud = dst;
}

AUTODRIVECORE_MODULE_END

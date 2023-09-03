#include "pch.h"
#include "ImageFilter.h"

namespace AutoDriveCode {
	namespace ImageFilter {
		using namespace std;
		using namespace cv;

		using namespace std;
		using namespace cv;


		Mat DrawStateInfoFilter(Mat& src, MachineStateType& stateInfo) {
			Mat dst = src.clone();
			Size size = src.size();

			Scalar colorBlack(0, 0, 0);
			Scalar colorWhite(255, 255, 255);
			Scalar colorBlue(200, 0, 0);
			Scalar colorRed(0, 0, 200);

			int frameRate = stateInfo.FrameDateTime.size();
			int speed = stateInfo.Rear.CurValue;
			float steerDegree = stateInfo.Steer.CurValue;
			float yawDegree = stateInfo.CameraYaw.CurValue;
			float pitchDegree = stateInfo.CameraPitch.CurValue;
			double distance = stateInfo.SonicSensor * 0.1;
			double leftFloorVal = stateInfo.FloorSensor[0];
			double centorFloorVal = stateInfo.FloorSensor[1];
			double rightFloorVal = stateInfo.FloorSensor[2];

			// 좌상단 숫자 정보
			{
				Point fpsStrLoc(20, 30);
				Point speedStrLoc(20, 60);
				Point steerStrLoc(20, 90);
				Point camYawStrLoc(20, 120);
				Point camPitchStrLoc(20, 150);

				string fpsStr = std::format("FPS : {:d}", frameRate);
				string speedStr = std::format("Speed : {:d}", speed);
				string steerStr = std::format("Steer : {:.01f}", steerDegree);
				string camYawStr = std::format("Yaw : {:.01f}", yawDegree);
				string camPitchStr = std::format("Pitch : {:.01f}", pitchDegree);

				putText(dst, fpsStr, fpsStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorBlack, 10);
				putText(dst, speedStr, speedStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorBlack, 10);
				putText(dst, steerStr, steerStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorBlack, 10);
				putText(dst, camYawStr, camYawStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorBlack, 10);
				putText(dst, camPitchStr, camPitchStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorBlack, 10);
				putText(dst, fpsStr, fpsStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorWhite, 3);
				putText(dst, speedStr, speedStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorWhite, 3);
				putText(dst, steerStr, steerStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorWhite, 3);
				putText(dst, camYawStr, camYawStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorWhite, 3);
				putText(dst, camPitchStr, camPitchStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorWhite, 3);
			}

			// 속도값
			{
				Size layoutSize(80, 200);
				int bgYOffset = size.height - 20 - layoutSize.height;
				int fgYOffset = size.height - 20 - layoutSize.height * 0.5;

				Rect speedBGRect(20, bgYOffset, layoutSize.width, layoutSize.height);
				Rect speedFGRect(20, fgYOffset, layoutSize.width, 0);

				rectangle(dst, speedBGRect, colorWhite, FILLED);
				if (speed >= 0)
				{
					speedFGRect.height = abs(speed * 0.02);
					speedFGRect.y = fgYOffset - speedFGRect.height;
					rectangle(dst, speedFGRect, colorRed, FILLED);
				}
				else
				{
					speedFGRect.y = fgYOffset;
					speedFGRect.height = abs(speed * 0.02);
					rectangle(dst, speedFGRect, colorBlue, FILLED);
				}
			}

			// 스티어러 각도
			{
				Point steerArrowFrom(200, size.height - 40);
				Point steerArrowVec(0, -140);

				double t_sin = sin(DEGREE_TO_RADIAN(steerDegree));
				double t_cos = cos(DEGREE_TO_RADIAN(steerDegree));

				Point newRotVec;
				newRotVec.x = steerArrowVec.x * t_cos - steerArrowVec.y * t_sin;
				newRotVec.y = steerArrowVec.x * t_sin + steerArrowVec.y * t_cos;
				Point steerArrowTo = steerArrowFrom + newRotVec;

				arrowedLine(dst, steerArrowFrom, steerArrowTo, colorBlack, 20);
				arrowedLine(dst, steerArrowFrom, steerArrowTo, colorWhite, 12);
			}

			// 카메라 위치 표시
			{
				Size bgSize(200, 160);
				double x_cos = sin(DEGREE_TO_RADIAN(yawDegree)) * 90.0;
				double y_cos = sin(DEGREE_TO_RADIAN(pitchDegree)) * 90.0;

				Rect camBGRect(size.width - bgSize.width - 20, size.height - bgSize.height - 20, bgSize.width, bgSize.height);
				Point camFGLoc(camBGRect.x + camBGRect.width * 0.5 + x_cos, camBGRect.y + camBGRect.height * 0.5 - y_cos + 10);

				rectangle(dst, camBGRect, colorWhite, FILLED);
				circle(dst, camFGLoc, 10, colorRed, FILLED);
			}

			// 음파 거리값 출력
			{
				Point distanceStrLoc(size.width * 0.5, size.height - 60);

				int baseline = 0;
				string distanceStr = std::format("{:05.02f}cm", distance);
				Size bgSize = getTextSize(distanceStr, FONT_HERSHEY_SIMPLEX, 0.8, 10, &baseline);
				Size fgSize = getTextSize(distanceStr, FONT_HERSHEY_SIMPLEX, 0.8, 3, &baseline);
				Point bgStrLoc = distanceStrLoc;
				Point fgStrLoc = distanceStrLoc;
				bgStrLoc.x -= bgSize.width * 0.5 - 8;
				fgStrLoc.x -= fgSize.width * 0.5 - 5;

				putText(dst, distanceStr, bgStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorBlack, 10);
				putText(dst, distanceStr, fgStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorWhite, 3);
			}

			// 하단 센서값
			{
				Point leftLoc(size.width * 0.5 - 30, size.height - 30);
				Point centerLoc(size.width * 0.5, size.height - 30);
				Point rightLoc(size.width * 0.5 + 30, size.height - 30);

				double multiLeftFloorVal = leftFloorVal / 1000.0;
				double multiCenterFloorVal = centorFloorVal / 1000.0;
				double multiRightFloorVal = rightFloorVal / 1000.0;

				if (multiLeftFloorVal > 1.0)
					multiLeftFloorVal = 1.0;
				if (multiCenterFloorVal > 1.0)
					multiCenterFloorVal = 1.0;
				if (multiRightFloorVal > 1.0)
					multiRightFloorVal = 1.0;

				Scalar leftColor(255 * multiLeftFloorVal, 255 * multiLeftFloorVal, 255);
				Scalar centerColor(255 * multiCenterFloorVal, 255 * multiLeftFloorVal, 255);
				Scalar rightColor(255 * multiRightFloorVal, 255 * multiLeftFloorVal, 255);

				circle(dst, leftLoc, 10, leftColor, FILLED);
				circle(dst, centerLoc, 10, centerColor, FILLED);
				circle(dst, rightLoc, 10, rightColor, FILLED);
			}

			return dst;
		}
		Mat CalibrationCameraFilter(Mat& src, CameraCaliDataType& caliData) {
			Mat dst = src;
			Size srcSize = src.size();

			// 외각에 생기는 bad pixel 자르기
			{
				int margin = caliData.Margin;
				Rect edgeCut(margin, margin, src.cols - margin * 2, src.rows - margin * 2);
				dst = dst(edgeCut).clone();
				srcSize = dst.size();
			}

			// 회전 및 빈공간 자르기
			if(false) {
				double rotDegree = caliData.RollDegree;
				Point2f rotCenter(srcSize.width * 0.5, srcSize.height * 0.5);
				Mat rotMat = getRotationMatrix2D(rotCenter, rotDegree, 1.0);
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

			// 단순 블러 + 노말
			{
				GaussianBlur(dst, dst, Size(), 1);
				normalize(dst, dst, 0, UCHAR_MAX, NORM_MINMAX);
			}

			return dst;
		}
		PTLCPtr ConvertImageToPointCloud(Mat& src, CameraCaliDataType& caliData, MachineStateType& stateInfo) {
			Mat t_src;

			// 외각에 생기는 bad pixel 자르기
			{
				int margin = caliData.Margin;
				Rect edgeCut(margin, margin, src.cols - margin * 2, src.rows - margin * 2);
				t_src = src(edgeCut).clone();
			}

			Size srcSize = caliData.PointCloudSize;
			resize(t_src, t_src, srcSize);

			float imgXDistanceRate = 1.0f;
			float imgYDistanceRate = 0.7f;

			float distance = 100.f;
			float x_scale = imgXDistanceRate / srcSize.width * distance;
			float z_scale = imgYDistanceRate / srcSize.height * distance;
			float x_offset = -srcSize.width * 0.5f;
			float z_offset = -srcSize.height * 0.5f;

			PTCPtr offsetPc = caliData.PointOffsets;
			PCL_NEW(PTLCType, srcPc);
			srcPc->resize(srcSize.area());
			for (int y = 0; y < srcSize.height; y++) {
				for (int x = 0; x < srcSize.width; x++) {
					size_t idx = y * srcSize.width + x;
					PTType& offset = offsetPc->at(idx);
					PTLType& pt = srcPc->at(idx);
					Vec3b& pxl = t_src.at<Vec3b>(idx);
					pt.x = offset.x;
					pt.y = offset.y;
					pt.z = offset.z;
					pt.b = pxl[0];
					pt.g = pxl[1];
					pt.r = pxl[2];
					pt.a = 255;
				}
			}

			Eigen::AngleAxisd roll(DEGREE_TO_RADIAN(caliData.RollDegree), -Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd yaw(DEGREE_TO_RADIAN(stateInfo.CameraYaw.CurValue), -Eigen::Vector3d::UnitZ());
			Eigen::AngleAxisd pitch(DEGREE_TO_RADIAN(stateInfo.CameraPitch.CurValue), Eigen::Vector3d::UnitX());
			Eigen::Matrix3d rotationMatrix = (roll * yaw * pitch).matrix();

			Eigen::Matrix4f transMat = Eigen::Matrix4f::Identity();
			for (int i = 0; i < 3; i++)for (int j = 0; j < 3; j++)
				transMat.col(i)[j] = rotationMatrix.col(i)[j];

			PCL_NEW(PTLCType, dstPC);
			pcl::transformPointCloud(*srcPc, *dstPC, transMat);

			return dstPC;
		}
	}
}

#include "pch.h"
#include "ImageFilter.h"

namespace AutoDriveCode {
	namespace ImageFilter {
		using namespace std;
		using namespace cv;

		using namespace std;
		using namespace cv;


		Mat DrawStateInfoFilter(Mat src, MachineStateType& stateInfo) {
			Mat dst = src.clone();
			Size size = src.size();

			Scalar colorBlack(0, 0, 0);
			Scalar colorWhite(255, 255, 255);
			Scalar colorBlue(200, 0, 0);
			Scalar colorRed(0, 0, 200);

			int frameRate = stateInfo.GetFPS();
			double temp = stateInfo.GetTemperature();
			int state = stateInfo.GetStateBits();
			int speed = stateInfo.GetRear().CurValue;
			float steerDegree = stateInfo.GetSteer().CurValue;
			float yawDegree = stateInfo.GetCameraYaw().CurValue;
			float pitchDegree = stateInfo.GetCameraPitch().CurValue;
			double distance = stateInfo.GetSonicValue() * 0.1;
			double leftFloorVal = stateInfo.GetFloorValue(0);
			double centorFloorVal = stateInfo.GetFloorValue(1);
			double rightFloorVal = stateInfo.GetFloorValue(2);

			// 좌상단 숫자 정보
			{
				Point fpsStrLoc(20, 30);
				Point tempStrLoc(20, 60);
				Point stateStrLoc(20, 90);
				Point speedStrLoc(20, 120);
				Point steerStrLoc(20, 150);
				Point camYawStrLoc(20, 180);
				Point camPitchStrLoc(20, 210);


				string fpsStr = std::format("FPS : {:d}", frameRate);
				string tempStr = std::format("Temp : {:.02f}", temp);
				string stateStr = std::format("State : {:05x}", state);
				string speedStr = std::format("Speed : {:d}", speed);
				string steerStr = std::format("Steer : {:.01f}", steerDegree);
				string camYawStr = std::format("Yaw : {:.01f}", yawDegree);
				string camPitchStr = std::format("Pitch : {:.01f}", pitchDegree);

				putText(dst, fpsStr, fpsStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorBlack, 10);
				putText(dst, speedStr, speedStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorBlack, 10);
				putText(dst, steerStr, steerStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorBlack, 10);
				putText(dst, camYawStr, camYawStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorBlack, 10);
				putText(dst, camPitchStr, camPitchStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorBlack, 10);
				putText(dst, fpsStr, fpsStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorWhite, 3);
				putText(dst, tempStr, tempStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorWhite, 3);
				putText(dst, stateStr, stateStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorWhite, 3);
				putText(dst, speedStr, speedStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorWhite, 3);
				putText(dst, steerStr, steerStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorWhite, 3);
				putText(dst, camYawStr, camYawStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorWhite, 3);
				putText(dst, camPitchStr, camPitchStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorWhite, 3);
			}

			// 속도값
			{
				Size layoutSize(80, 200);
				int bgYOffset = size.height - 20 - layoutSize.height;
				int fgYOffset = size.height - 20 - layoutSize.height * 0.5;

				Rect speedBGRect(20, bgYOffset, layoutSize.width, layoutSize.height);
				Rect speedFGRect(20, fgYOffset, layoutSize.width, abs(speed / 20.f));

				rectangle(dst, speedBGRect, colorWhite, FILLED);
				if (speed >= 0)
				{
					speedFGRect.y = fgYOffset - speedFGRect.height;
					rectangle(dst, speedFGRect, colorRed, FILLED);
				}
				else
				{
					speedFGRect.y = fgYOffset;
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
				Size bgSize = getTextSize(distanceStr, FONT_HERSHEY_DUPLEX, 0.8, 10, &baseline);
				Size fgSize = getTextSize(distanceStr, FONT_HERSHEY_DUPLEX, 0.8, 3, &baseline);
				Point bgStrLoc = distanceStrLoc;
				Point fgStrLoc = distanceStrLoc;
				bgStrLoc.x -= bgSize.width * 0.5 - 8;
				fgStrLoc.x -= fgSize.width * 0.5 - 5;

				putText(dst, distanceStr, bgStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorBlack, 10);
				putText(dst, distanceStr, fgStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorWhite, 3);
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
		Mat CalibrationCameraFilter(Mat src, int margin, double rotDeg) {
			Mat dst = src;
			Size srcSize = src.size();

			// 외각에 생기는 bad pixel 자르기
			{
				Rect edgeCut(margin, margin, src.cols - margin * 2, src.rows - margin * 2);
				dst = dst(edgeCut).clone();
				srcSize = dst.size();
			}

			// 회전 및 빈공간 자르기
			{
				Point2f rotCenter(srcSize.width * 0.5, srcSize.height * 0.5);
				Mat rotMat = getRotationMatrix2D(rotCenter, rotDeg, 1.0);
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

		////////////////////////////////////////////////////////////////////////////////

		PTLCPtr ConvertImageToPointCloud(Mat src, int margin, Size resizedSize, PTCPtr caliOffset, float pitchDeg, float yawDeg) {
			// 외각에 생기는 bad pixel 자르기
			Mat cutImg;
			{
				Rect edgeCut(margin, margin, src.cols - margin * 2, src.rows - margin * 2);
				cutImg = src(edgeCut);
			}

			Mat resizedImg;
			{
				resize(cutImg, resizedImg, resizedSize);
			}

			float imgXDistanceRate = 1.0f;
			float imgYDistanceRate = 0.7f;

			float distance = 100.f;
			float x_scale = imgXDistanceRate / resizedSize.width * distance;
			float z_scale = imgYDistanceRate / resizedSize.height * distance;
			float x_offset = -resizedSize.width * 0.5f;
			float z_offset = -resizedSize.height * 0.5f;

			PCL_NEW(PTLCType, srcPc);
			srcPc->resize(resizedSize.area());
			for (int y = 0; y < resizedSize.height; y++) {
				for (int x = 0; x < resizedSize.width; x++) {
					size_t idx = y * resizedSize.width + x;
					PTType& offset = caliOffset->at(idx);
					PTLType& pt = srcPc->at(idx);
					Vec3b& pxl = resizedImg.at<Vec3b>(idx);
					pt.x = offset.x;
					pt.y = offset.y;
					pt.z = offset.z;
					pt.b = pxl[0];
					pt.g = pxl[1];
					pt.r = pxl[2];
					pt.a = 255;
				}
			}

			Eigen::AngleAxisd yaw(DEGREE_TO_RADIAN(yawDeg), -Eigen::Vector3d::UnitZ());
			Eigen::AngleAxisd pitch(DEGREE_TO_RADIAN(pitchDeg), Eigen::Vector3d::UnitX());
			//Eigen::AngleAxisd roll(DEGREE_TO_RADIAN(caliData.RollDegree), -Eigen::Vector3d::UnitY());
			Eigen::Matrix3d rotationMatrix = (yaw * pitch).matrix();

			Eigen::Matrix4f transMat = Eigen::Matrix4f::Identity();
			for (int i = 0; i < 3; i++)for (int j = 0; j < 3; j++)
				transMat.col(i)[j] = rotationMatrix.col(i)[j];

			PCL_NEW(PTLCType, dstPC);
			pcl::transformPointCloud(*srcPc, *dstPC, transMat);

			return dstPC;
		}
	}
}

#include "pch.h"
#include "ImageFilter.h"

namespace AutoDriveCode {
	namespace ImageFilter {
		using namespace std;
		using namespace cv;

		Mat ApplyStateInfo(Mat& src, StateType& stateInfo) {
			Mat dst = src.clone();
			Size size = src.size();

			Scalar colorBlack(0, 0, 0);
			Scalar colorWhite(255, 255, 255);
			Scalar colorBlue(200, 0, 0);
			Scalar colorRed(0, 0, 200);

			int speed = stateInfo.Rear.CurValue;
			float steerDegree = stateInfo.Steer.CurValue;
			float yawDegree = stateInfo.CameraYaw.CurValue;
			float pitchDegree = stateInfo.CameraPitch.CurValue;
			double distance = stateInfo.SonicSensor * 0.1;
			double leftFloorVal = stateInfo.FloorSensor[0];
			double centorFloorVal = stateInfo.FloorSensor[1];
			double rightFloorVal = stateInfo.FloorSensor[2];

			{
				Point speedStrLoc(20, 30);
				Point steerStrLoc(20, 60);
				Point camYawStrLoc(20, 90);
				Point camPitchStrLoc(20, 120);

				string speedStr = std::format("Speed : {:d}", speed);
				string steerStr = std::format("Steer : {:.01f}", steerDegree);
				string camYawStr = std::format("Yaw : {:.01f}", yawDegree);
				string camPitchStr = std::format("Pitch : {:.01f}", pitchDegree);

				putText(dst, speedStr, speedStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorBlack, 10);
				putText(dst, steerStr, steerStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorBlack, 10);
				putText(dst, camYawStr, camYawStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorBlack, 10);
				putText(dst, camPitchStr, camPitchStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorBlack, 10);
				putText(dst, speedStr, speedStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorWhite, 3);
				putText(dst, steerStr, steerStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorWhite, 3);
				putText(dst, camYawStr, camYawStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorWhite, 3);
				putText(dst, camPitchStr, camPitchStrLoc, FONT_HERSHEY_SIMPLEX, 0.8, colorWhite, 3);
			}
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
			{
				Size bgSize(200, 160);
				double x_cos = sin(DEGREE_TO_RADIAN(yawDegree)) * 90.0;
				double y_cos = sin(DEGREE_TO_RADIAN(pitchDegree)) * 90.0;

				Rect camBGRect(size.width - bgSize.width - 20, size.height - bgSize.height - 20, bgSize.width, bgSize.height);
				Point camFGLoc(camBGRect.x + camBGRect.width * 0.5 + x_cos, camBGRect.y + camBGRect.height * 0.5 - y_cos + 10);

				rectangle(dst, camBGRect, colorWhite, FILLED);
				circle(dst, camFGLoc, 10, colorRed, FILLED);
			}
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
		Mat ApplyAdaptiveBrightness(Mat& src) {
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

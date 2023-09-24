#include "pch.h"
#include "MachineDebugImageFilter.h"


AUTODRIVECORE_MODULE_START

using namespace std;
using namespace cv;

MachineDebugImageFilterType::MachineDebugImageFilterType() {
	colorBlack = Scalar(0, 0, 0);
	colorWhite = Scalar(255, 255, 255);
	colorBlue = Scalar(200, 0, 0);
	colorRed = Scalar(0, 0, 200);
	fpsStrLoc = Point(20, 30);
	tempStrLoc = Point(20, 60);
	stateStrLoc = Point(20, 90);
	speedStrLoc = Point(20, 120);
	steerStrLoc = Point(20, 150);
	camYawStrLoc = Point(20, 180);
	camPitchStrLoc = Point(20, 210);
	powerLayoutSize = Size(80, 200);
}
void MachineDebugImageFilterType::SetWithMachineState(MachineStateType& _machine_state) {
	MachineStateType cloned_state;
	_machine_state.Clone(cloned_state);

	m_FPS = cloned_state.GetFPS();
	m_Temperature = cloned_state.GetTemperature();
	m_StateByte = cloned_state.GetStateBits();
	m_RearPower = cloned_state.GetRear().CurValue;
	m_SteerDegree = cloned_state.GetSteer().CurValue;
	m_CameraYawDegree = cloned_state.GetCameraYaw().CurValue;
	m_CameraPitchDegree = cloned_state.GetCameraPitch().CurValue;
	m_SonicDistance = cloned_state.GetSonicValue() * 0.1;
	m_LeftFloorVal = cloned_state.GetFloorValue(0);
	m_CentorFloorVal = cloned_state.GetFloorValue(1);
	m_RightFloorVal = cloned_state.GetFloorValue(2);
}
void MachineDebugImageFilterType::Update() {
	Mat dst = m_Inplace ? m_InputImage : m_InputImage.clone();
	Size size = dst.size();

	// 좌상단 숫자 정보
	{
		string fpsStr = std::format("FPS : {:d}", m_FPS);
		string tempStr = std::format("Temp : {:.02f}", m_Temperature);
		string stateStr = std::format("State : {:05x}", m_StateByte);
		string speedStr = std::format("Speed : {:d}", m_RearPower);
		string steerStr = std::format("Steer : {:.01f}", m_SteerDegree);
		string camYawStr = std::format("Yaw : {:.01f}", m_CameraYawDegree);
		string camPitchStr = std::format("Pitch : {:.01f}", m_CameraPitchDegree);

		putText(dst, fpsStr, fpsStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorBlack, 10);
		putText(dst, tempStr, tempStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorBlack, 10);
		putText(dst, stateStr, stateStrLoc, FONT_HERSHEY_DUPLEX, 0.8, colorBlack, 10);
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
		int bgYOffset = size.height - 20 - powerLayoutSize.height;
		int fgYOffset = size.height - 20 - powerLayoutSize.height * 0.5;

		Rect speedBGRect(20, bgYOffset, powerLayoutSize.width, powerLayoutSize.height);
		Rect speedFGRect(20, fgYOffset, powerLayoutSize.width, abs(m_RearPower / 20.f));

		rectangle(dst, speedBGRect, colorWhite, FILLED);
		if (m_RearPower >= 0)
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

		double t_sin = sin(DEGREE_TO_RADIAN(m_SteerDegree));
		double t_cos = cos(DEGREE_TO_RADIAN(m_SteerDegree));

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
		double x_cos = sin(DEGREE_TO_RADIAN(m_CameraYawDegree)) * 90.0;
		double y_cos = sin(DEGREE_TO_RADIAN(m_CameraPitchDegree)) * 90.0;

		Rect camBGRect(size.width - bgSize.width - 20, size.height - bgSize.height - 20, bgSize.width, bgSize.height);
		Point camFGLoc(camBGRect.x + camBGRect.width * 0.5 + x_cos, camBGRect.y + camBGRect.height * 0.5 - y_cos + 10);

		rectangle(dst, camBGRect, colorWhite, FILLED);
		circle(dst, camFGLoc, 10, colorRed, FILLED);
	}

	// 음파 거리값 출력
	{
		Point distanceStrLoc(size.width * 0.5, size.height - 60);

		int baseline = 0;
		string distanceStr = std::format("{:05.02f}cm", m_SonicDistance);
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

		double multiLeftFloorVal = m_LeftFloorVal / 1000.0;
		double multiCenterFloorVal = m_CentorFloorVal / 1000.0;
		double multiRightFloorVal = m_RightFloorVal / 1000.0;

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

	m_OutputImage = dst;
}

AUTODRIVECORE_MODULE_END


#pragma once
#include "TypeToTypeFilter.h"


AUTODRIVECORE_MODULE_START

class MachineDebugImageFilterType
	: public ImageToImageFilterType
{
	AUTODRIVECORE_CLASS_GET_SET(int, FPS);
	AUTODRIVECORE_CLASS_GET_SET(double, Temperature);
	AUTODRIVECORE_CLASS_GET_SET(int, StateByte);
	AUTODRIVECORE_CLASS_GET_SET(int, RearPower);
	AUTODRIVECORE_CLASS_GET_SET(float, SteerDegree);
	AUTODRIVECORE_CLASS_GET_SET(float, CameraYawDegree);
	AUTODRIVECORE_CLASS_GET_SET(float, CameraPitchDegree);
	AUTODRIVECORE_CLASS_GET_SET(double, SonicDistance);
	AUTODRIVECORE_CLASS_GET_SET(double, LeftFloorVal);
	AUTODRIVECORE_CLASS_GET_SET(double, CentorFloorVal);
	AUTODRIVECORE_CLASS_GET_SET(double, RightFloorVal);

protected:
	cv::Scalar colorBlack;
	cv::Scalar colorWhite;
	cv::Scalar colorBlue;
	cv::Scalar colorRed;
	cv::Point fpsStrLoc;
	cv::Point tempStrLoc;
	cv::Point stateStrLoc;
	cv::Point speedStrLoc;
	cv::Point steerStrLoc;
	cv::Point camYawStrLoc;
	cv::Point camPitchStrLoc;
	cv::Size powerLayoutSize;

public:
	MachineDebugImageFilterType();
	void SetWithMachineState(MachineStateType& state);
	virtual void Update();
};

AUTODRIVECORE_MODULE_END

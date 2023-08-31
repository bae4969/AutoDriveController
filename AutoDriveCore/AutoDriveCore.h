#pragma once
#include "Defs.h"
#include "Types.h"

namespace AutoDriveCode {
	AUTODRIVECORE_EXPORT void Init(char* pubAddress, char* subAddress);
	AUTODRIVECORE_EXPORT void Release();

	AUTODRIVECORE_EXPORT ImageData GetOriginImage();
	AUTODRIVECORE_EXPORT ImageData GetStateImage();
	AUTODRIVECORE_EXPORT ImageData GetFilterImage();

	AUTODRIVECORE_EXPORT void TurnOff();
	AUTODRIVECORE_EXPORT void StopMove();
	AUTODRIVECORE_EXPORT void ChangeRearValue(int diff);
	AUTODRIVECORE_EXPORT void ChangeSteerValue(float diff);
	AUTODRIVECORE_EXPORT void ChangeCameraPitchValue(float diff);
	AUTODRIVECORE_EXPORT void ChangeCameraPitchYaw(float diff);
}



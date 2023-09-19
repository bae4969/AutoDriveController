#pragma once
#include "Defs.h"
#include "Types.h"

namespace AutoDriveCode {
	AUTODRIVECORE_EXPORT void Init(char* pubAddress, char* subAddress, void* handle);
	AUTODRIVECORE_EXPORT void Release();

	AUTODRIVECORE_EXPORT ImageData GetStateImage();

	AUTODRIVECORE_EXPORT void ExecuteEvent(char* eventName);

	AUTODRIVECORE_EXPORT void TurnOff();
	AUTODRIVECORE_EXPORT void StopMove();
	AUTODRIVECORE_EXPORT void ChangeRearValue(int diff);
	AUTODRIVECORE_EXPORT void ChangeSteerValue(float diff);
	AUTODRIVECORE_EXPORT void ChangeCameraPitchValue(float diff);
	AUTODRIVECORE_EXPORT void ChangeCameraYawValue(float diff);
}



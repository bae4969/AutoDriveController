#pragma once
#include "Defs.h"
#include "Types.h"

namespace AutoDriveCode {
	AUTODRIVECORE_EXPORT void Init(char* pubAddress, char* subAddress);
	AUTODRIVECORE_EXPORT void Release();

	AUTODRIVECORE_EXPORT ImageData GetOriginImage();
	AUTODRIVECORE_EXPORT ImageData GetFilterImage();
}



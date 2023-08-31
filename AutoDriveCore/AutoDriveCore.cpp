#include "pch.h"
#include "AutoDriveCore.h"
#include "Switcher.h"

namespace AutoDriveCode {
	using namespace std;
	using namespace cv;


	Switcher g_Switcher;
	ImageData g_OriginImage;
	ImageData g_StateImage;
	ImageData g_FilterImage;


	void Init(char* pubAddress, char* subAddress) {
		g_Switcher.Init(pubAddress, subAddress);
	}
	void Release() {
		g_Switcher.Release();
		g_OriginImage.Release();
		g_FilterImage.Release();
	}

	ImageData GetOriginImage() {
		g_Switcher.GetOriginImage(g_OriginImage);
		return g_OriginImage;
	}
	ImageData GetStateImage() {
		g_Switcher.GetStateImage(g_StateImage);
		return g_StateImage;
	}
	ImageData GetFilterImage() {
		g_Switcher.GetFilterImage(g_FilterImage);
		return g_FilterImage;
	}

	void TurnOff() {
		g_Switcher.TurnOff();
	}
	void StopMove() {
		g_Switcher.StopMove();
	}
	void ChangeRearValue(int diff) {
		g_Switcher.ChangeRearValue(diff);
	}
	void ChangeSteerValue(float diff) {
		g_Switcher.ChangeSteerValue(diff);
	}
	void ChangeCameraPitchValue(float diff) {
		g_Switcher.ChangeCameraPitchValue(diff);
	}
	void ChangeCameraPitchYaw(float diff) {
		g_Switcher.ChangeCameraPitchYaw(diff);
	}
}

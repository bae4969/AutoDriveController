#include "pch.h"
#include "AutoDriveCore.h"
#include "Switcher.h"

namespace AutoDriveCode {
	using namespace std;
	using namespace cv;


	Switcher g_Switcher;
	ImageData g_StateImage;


	void Init(char* pubAddress, char* subAddress, void* handle) {
		g_Switcher.Init(pubAddress, subAddress, handle);
	}
	void Release() {
		g_Switcher.Release();
		g_StateImage.Release();
	}

	ImageData GetStateImage() {
		g_Switcher.GetStateImage(g_StateImage);
		return g_StateImage;
	}

	void ExecuteEvent(char* eventName) {
		string eventStr = eventName;

		if (eventStr == "RESIZE_VISUALIZER") {
			g_Switcher.ExecuteEventResizeVisualizer();
		}
		else if (eventStr == "PUSH_IMAGE_PC") {
			g_Switcher.ExecuteEventPushImagePointCloud();
		}
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
	void ChangeCameraYawValue(float diff) {
		g_Switcher.ChangeCameraYawValue(diff);
	}
}

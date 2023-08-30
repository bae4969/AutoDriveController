#include "pch.h"
#include "AutoDriveCore.h"
#include "Switcher.h"

namespace AutoDriveCode {
	using namespace std;
	using namespace cv;


	Switcher g_Switcher;
	ImageData g_OriginImage;
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
	ImageData GetFilterImage() {
		g_Switcher.GetFilterImage(g_FilterImage);
		return g_FilterImage;
	}
}

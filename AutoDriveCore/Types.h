#pragma once
#include "Defs.h"
#include <opencv2/core.hpp>

namespace AutoDriveCode {
	struct ImageData {
		int W;
		int H;
		int Ch;
		int Step;
		void* Data;

		ImageData();
		ImageData Clone();
		void Update(cv::Mat mat);
		void Release();
	};

	template<typename TYPE>
	struct MotorStateType {
		TYPE CurValue;
		TYPE TarValue;
		TYPE Speed;
	};
	struct StateType {
		std::mutex syncMutex;

		MotorStateType<int> Rear;
		MotorStateType<float> Steer;
		MotorStateType<float> CameraPitch;
		MotorStateType<float> CameraYaw;

		double SonicSensor = 0.0;
		std::vector<int> FloorSensor = std::vector<int>(3, 0);

		std::queue<std::chrono::steady_clock::time_point> FrameDateTime;
		cv::Mat OriginImage;
		cv::Mat StateImage;
		cv::Mat FilterImage;


		void Clone(StateType& state);
		void UpdateMoveMotorState(MotorStateType<int>& rear, MotorStateType<float>& steer);
		void UpdateCameraMotorState(MotorStateType<float>& cameraPitch, MotorStateType<float>& cameraYaw);
		void UpdateSensorState(double& sonicSensor, std::vector<int>& floorSensor);
		void UpdateCameraImage(cv::Mat& originImage, cv::Mat& stateImage, cv::Mat& filterImage);

		cv::Mat GetOriginImage();
		cv::Mat GetStateImage();
		cv::Mat GetFilterImage();

		MotorStateType<int> GetRear();
		MotorStateType<float> GetSteer();
		MotorStateType<float> GetCameraPitch();
		MotorStateType<float> GetCameraYaw();
	};
}



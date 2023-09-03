#pragma once
#include "Defs.h"
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace AutoDriveCode {
	typedef pcl::PointXYZ PTType;
	typedef pcl::PointNormal PTNType;
	typedef pcl::PointXYZRGB PTLType;
	typedef pcl::PointXYZRGBNormal PTLNType;

	typedef pcl::PointCloud<PTType> PTCType;
	typedef pcl::PointCloud<PTNType> PTNCType;
	typedef pcl::PointCloud<PTLType> PTLCType;
	typedef pcl::PointCloud<PTLNType> PTLNCType;

	typedef PTCType::Ptr PTCPtr;
	typedef PTNCType::Ptr PTNCPtr;
	typedef PTLCType::Ptr PTLCPtr;
	typedef PTLNCType::Ptr PTLNCPtr;

	struct ImageData {
		int W;
		int H;
		int Ch;
		int Step;
		void* Data;

		ImageData();
		ImageData Clone();
		void Update(cv::Mat& mat);
		void Release();
	};

	template<typename TYPE>
	struct MotorStateType {
		TYPE CurValue;
		TYPE TarValue;
		TYPE Speed;
	};
	struct MachineStateType {
		std::mutex syncMutex;

		MotorStateType<int> Rear;
		MotorStateType<float> Steer;
		MotorStateType<float> CameraPitch;
		MotorStateType<float> CameraYaw;

		double SonicSensor = 0.0;
		std::vector<int> FloorSensor = std::vector<int>(3, 0);

		std::queue<std::chrono::steady_clock::time_point> FrameDateTime;


		void Clone(MachineStateType& state);
		void UpdateMoveMotorState(MotorStateType<int>& rear, MotorStateType<float>& steer);
		void UpdateCameraMotorState(MotorStateType<float>& cameraPitch, MotorStateType<float>& cameraYaw);
		void UpdateSensorState(double& sonicSensor, std::vector<int>& floorSensor);
		void UpdateFPS();

		MotorStateType<int> GetRear();
		MotorStateType<float> GetSteer();
		MotorStateType<float> GetCameraPitch();
		MotorStateType<float> GetCameraYaw();
	};
	struct ImageStateType {
		std::mutex syncMutex;

		cv::Mat OriginImage;
		cv::Mat StateImage;
		cv::Mat FilterImage;
		PTLCPtr PointCloud;


		void UpdateOriginImage(cv::Mat& originImage);
		void UpdateStateImage(cv::Mat& stateImage);
		void UpdateFilterImage(cv::Mat& filterImage);
		void UpdatePointCloud(PTLCPtr& pointCloud);

		cv::Mat GetOriginImage();
		cv::Mat GetStateImage();
		cv::Mat GetFilterImage();
		PTLCPtr GetPointCloud();
	};

	struct CameraCaliDataType {
		int Margin;
		double RollDegree;
		cv::Size PointCloudSize;
		PTCPtr PointOffsets;

		void Init();
	};
}



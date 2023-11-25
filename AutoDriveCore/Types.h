#pragma once
#include "Defs.h"
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace AutoDriveCode {
	typedef std::chrono::steady_clock ClockType;
	typedef ClockType::time_point TimePointType;

	typedef pcl::PointXYZ PTType;
	typedef pcl::Normal NorType;
	typedef pcl::PointNormal PTNType;
	typedef pcl::PointXYZRGB PTLType;
	typedef pcl::PointXYZRGBNormal PTLNType;
	typedef pcl::PFHRGBSignature250 RgbFType;

	typedef pcl::PointCloud<PTType> PTCType;
	typedef pcl::PointCloud<NorType> NorCType;
	typedef pcl::PointCloud<PTNType> PTNCType;
	typedef pcl::PointCloud<PTLType> PTLCType;
	typedef pcl::PointCloud<PTLNType> PTLNCType;
	typedef pcl::PointCloud<RgbFType> RgbFCType;

	typedef PTCType::Ptr PTCPtr;
	typedef NorCType::Ptr NorCPtr;
	typedef PTNCType::Ptr PTNCPtr;
	typedef PTLCType::Ptr PTLCPtr;
	typedef PTLNCType::Ptr PTLNCPtr;
	typedef RgbFCType::Ptr RgbFCPtr;

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
	class MachineStateType {
	private:
		std::mutex syncMutex;

		MotorStateType<int> Rear;
		MotorStateType<float> Steer;
		MotorStateType<float> CameraPitch;
		MotorStateType<float> CameraYaw;

		double SonicSensor = 0.0;
		std::vector<int> FloorSensor = std::vector<int>(3, 0);

		double MachineTemperature;
		int MachineStateBits;


		std::queue<ClockType::time_point> FrameDateTime;

	public:
		void Clone(MachineStateType& state);
		void UpdateMoveMotorState(MotorStateType<int>& rear, MotorStateType<float>& steer);
		void UpdateCameraMotorState(MotorStateType<float>& cameraPitch, MotorStateType<float>& cameraYaw);
		void UpdateSensorState(double& sonicSensor, std::vector<int>& floorSensor);
		void UpdateLcdState(double& temp, int& state);
		void UpdateFPS();

		MotorStateType<int> GetRear();
		MotorStateType<float> GetSteer();
		MotorStateType<float> GetCameraPitch();
		MotorStateType<float> GetCameraYaw();
		double GetSonicValue();
		int GetFloorValue(int idx);
		int GetFPS();
		double GetTemperature();
		int GetStateBits();
	};

	template<typename TYPE>
	class DeltaType {
	private:
		std::mutex syncMutex;
		TimePointType Time = TimePointType::min();
		TYPE Data;

	public:
		void Set(DeltaType<TYPE>& deltaPc);
		void Set(TYPE pc);
		void Get(TimePointType& time, TYPE& pc);
	};

	class ConclusionType {
	private:
		std::mutex syncMutex;

	};
}



#pragma once
#include "Types.h"
#include <zmq_addon.hpp>
#include <opencv2/core.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace AutoDriveCode {
	class Switcher {
		enum CMD_VALUE_TYPE {
			CMD_VALUE_TYPE_TARGET,
			CMD_VALUE_TYPE_SPEED,
		};

		bool m_isStop;
		std::shared_ptr<std::thread> m_subThread;
		std::shared_ptr<std::thread> m_pubThread;
		std::shared_ptr<std::thread> m_updateConnectionThread;
		std::shared_ptr<std::thread> m_updateStateThread;
		std::shared_ptr<std::thread> m_updateLidarThread;
		std::shared_ptr<std::thread> m_updateImageThread;
		std::shared_ptr<std::thread> m_updateConclusionThread;

		std::string m_subAddress;
		std::string m_pubAddress;
		pcl::visualization::PCLVisualizer::Ptr m_viewer;

		std::mutex m_queueCmdMutex;
		std::mutex m_queueStateMutex;
		std::mutex m_queueLidarMutex;
		std::mutex m_queueImageMutex;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueCmd;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueState;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueLidar;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueImage;

		MachineStateType m_currentMachineState;
		DeltaType<cv::Mat> m_currentStateImage;
		DeltaType<PTLNCPtr> m_currentImagePc;
		DeltaType<PTNCPtr> m_currentLidarPc;

		std::queue<PTLNCPtr> m_imagePcBuffer;
		DeltaType<PTLNCPtr> m_srcCameraPc;
		DeltaType<PTNCPtr> m_srcLidarPc;


		void subscribeThreadFunc();
		void publishThreadFunc();
		void updateConnectionThreadFunc();
		void updateStateThreadFunc();
		void updateImageThreadFunc();
		void updateLidarThreadFunc();
		void updateConclusionThreadFunc();

	public:
		void Init(std::string pubAddress, std::string subAddress, void* pcWndHandle);
		void Release();

		void GetStateImage(ImageData& imgData);
		void ExecuteEventResizeVisualizer();
		void ExecuteEventPushImagePointCloud();
		void ExecuteEventPopAllImagePointCloud();


		void TurnOff();
		void StopMove();
		void SetRearValue(CMD_VALUE_TYPE type, int value);
		void SetSteerValue(CMD_VALUE_TYPE type, float value);
		void SetCameraPitchValue(CMD_VALUE_TYPE type, float value);
		void SetCameraYawValue(CMD_VALUE_TYPE type, float value);

		void ChangeRearValue(int diff);
		void ChangeSteerValue(float diff);
		void ChangeCameraPitchValue(float diff);
		void ChangeCameraYawValue(float diff);
	};
}


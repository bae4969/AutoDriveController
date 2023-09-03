#pragma once
#include "Types.h"
#include <zmq_addon.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace AutoDriveCode {
	class Switcher {
		enum CMD_VALUE_TYPE {
			CMD_VALUE_TYPE_TARGET,
			CMD_VALUE_TYPE_SPEED,
		};

		bool m_isStop;
		std::thread m_subThread;
		std::thread m_pubThread;
		std::thread m_updateConnectionThread;
		std::thread m_updateStateThread;
		std::thread m_updateImageThread;
		std::thread m_updateViewerThread;

		std::string m_subAddress;
		std::string m_pubAddress;

		std::mutex m_queueCmdMutex;
		std::mutex m_queueStateMutex;
		std::mutex m_queueImageMutex;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueCmd;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueState;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueImage;

		MachineStateType m_currentMachineState;
		ImageStateType m_currentCameraState;
		CameraCaliDataType m_cameraCaliData;

		std::mutex m_viwerMutex;
		pcl::visualization::PCLVisualizer::Ptr m_viewer;

		bool m_isBusy;
		std::mutex m_pipelineMutex[4];


		void subscribeThreadFunc();
		void publishThreadFunc();
		void updateConnectionThreadFunc();
		void updateStateThreadFunc();
		void updateImageThreadFunc();
		void updateViewerThreadFunc();
		void updatePipelineThreadFunc(cv::Mat originImage);

	public:
		void Init(std::string pubAddress, std::string subAddress);
		void Release();

		void GetOriginImage(ImageData& imgData);
		void GetStateImage(ImageData& imgData);
		void GetFilterImage(ImageData& imgData);

		void SetPointCloudViwerWindow(void* handle);

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


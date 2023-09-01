#pragma once
#include "Types.h"
#include <zmq_addon.hpp>

namespace AutoDriveCode {
	class Switcher {
		bool m_isStop;
		std::thread m_subThread;
		std::thread m_pubThread;
		std::thread m_updateConnectionThread;
		std::thread m_updateStateThread;
		std::thread m_updateImageThread;

		std::string m_subAddress;
		std::string m_pubAddress;

		std::mutex m_queueCmdMutex;
		std::mutex m_queueStateMutex;
		std::mutex m_queueImageMutex;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueCmd;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueState;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueImage;
		StateType m_currentState;


		void subscribeThreadFunc();
		void publishThreadFunc();
		void updateConnectionThreadFunc();
		void updateStateThreadFunc();
		void updateImageThreadFunc();
		void updateStateImageThreadFunc(cv::Mat* stateImage, cv::Mat* originImage);
		void updateFilterImageThreadFunc(cv::Mat* filterImage, cv::Mat* originImage);

	public:
		void Init(std::string pubAddress, std::string subAddress);
		void Release();

		void GetOriginImage(ImageData& imgData);
		void GetStateImage(ImageData& imgData);
		void GetFilterImage(ImageData& imgData);

		void TurnOff();
		void StopMove();
		void ChangeRearValue(int diff);
		void ChangeSteerValue(float diff);
		void ChangeCameraPitchValue(float diff);
		void ChangeCameraPitchYaw(float diff);
	};
}


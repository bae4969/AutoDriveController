#pragma once
#include "Types.h"
#include <zmq_addon.hpp>

namespace AutoDriveCode {
	class Switcher {
		bool m_isStop;
		std::thread m_pubThread;
		std::thread m_subStateThread;
		std::thread m_subImageThread;
		std::thread m_updateConnectionThread;

		std::string m_subAddress;
		std::string m_pubAddress;

		std::mutex m_queueStateMutex;
		std::mutex m_queueCmdMutex;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueState;
		std::queue<std::shared_ptr<zmq::multipart_t>> m_queueCmd;
		StateType m_currentState;


		void publishThreadFunc();
		void subscribeStateThreadFunc();
		void subscribeImageThreadFunc();
		void updateConnectionThreadFunc();

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


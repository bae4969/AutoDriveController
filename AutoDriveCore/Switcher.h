#pragma once
#include "Types.h"
#include <zmq_addon.hpp>

namespace AutoDriveCode {
	class Switcher {
		bool m_isStop;
		std::thread m_pubThread;
		std::thread m_subStateThread;
		std::thread m_subImageThread;

		std::string m_subAddress;
		std::string m_pubAddress;

		std::mutex m_queueStateMutex;
		std::mutex m_queueCmdMutex;
		std::queue<zmq::multipart_t> m_queueState;
		std::queue<zmq::multipart_t> m_queueCmd;
		StateType m_currentState;


		void publishThreadFunc();
		void subscribeStateThreadFunc();
		void subscribeImageThreadFunc();

	public:
		void Init(std::string pubAddress, std::string subAddress);
		void Release();

		void GetOriginImage(ImageData& imgData);
		void GetFilterImage(ImageData& imgData);
	};
}


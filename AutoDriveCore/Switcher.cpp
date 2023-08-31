#include "pch.h"
#include "Switcher.h"
#include "ImageFilter.h"

namespace AutoDriveCode {
	using namespace std;
	using namespace cv;

	void Switcher::Init(string pubAddress, string subAddress) {
		m_isStop = false;
		m_pubAddress = pubAddress;
		m_subAddress = subAddress;

		m_pubThread = thread(&Switcher::publishThreadFunc, this);
		m_subStateThread = thread(&Switcher::subscribeStateThreadFunc, this);
		m_subImageThread = thread(&Switcher::subscribeImageThreadFunc, this);
		m_updateConnectionThread = thread(&Switcher::updateConnectionThreadFunc, this);
	}
	void Switcher::Release() {
		m_isStop = true;
		m_pubThread.join();
		m_subStateThread.join();
		m_subImageThread.join();
		m_updateConnectionThread.join();
	}

	void Switcher::publishThreadFunc() {
		zmq::context_t context(1);
		zmq::socket_t pubSocket(context, ZMQ_PUB);

		pubSocket.set(zmq::sockopt::sndhwm, 20);
		pubSocket.set(zmq::sockopt::sndtimeo, 10);
		pubSocket.connect(m_subAddress);

		while (!m_isStop) {
			try {
				bool isContinue = false;
				zmq::multipart_t msg;

				m_queueCmdMutex.lock();
				size_t cmdCount = m_queueCmd.size();
				if (cmdCount == 0) {
					isContinue = true;
				}
				else if (cmdCount > 4) {
					isContinue = true;
					m_queueCmd.pop();
				}
				else {
					msg = m_queueCmd.front()->clone();
					m_queueCmd.pop();
				}
				m_queueCmdMutex.unlock();

				if (isContinue) {
					this_thread::sleep_for(chrono::milliseconds(10));
					continue;
				}

				msg.send(pubSocket);
			}
			catch (...) {}
		}

		pubSocket.disconnect(m_subAddress);
	}
	void Switcher::subscribeStateThreadFunc() {
		zmq::context_t context(1);
		zmq::socket_t subSocket(context, ZMQ_SUB);

		subSocket.set(zmq::sockopt::rcvhwm, 20);
		subSocket.set(zmq::sockopt::rcvtimeo, 10);
		subSocket.set(zmq::sockopt::subscribe, "STATE_MOVE_MOTOR");
		subSocket.set(zmq::sockopt::subscribe, "STATE_CAMERA_MOTOR");
		subSocket.set(zmq::sockopt::subscribe, "STATE_SENSOR");
		subSocket.connect(m_pubAddress);

		while (!m_isStop) {
			try {
				zmq::multipart_t msg;
				if (msg.recv(subSocket)) {
					string topic = msg.popstr();
					if (topic == "STATE_MOVE_MOTOR" && msg.size() == 6) {
						MotorStateType<int> t_rear;
						MotorStateType<float> t_steer;
						memcpy_s(&t_rear.CurValue, sizeof(t_rear.CurValue), msg[0].data(), msg[0].size());
						memcpy_s(&t_rear.TarValue, sizeof(t_rear.TarValue), msg[1].data(), msg[1].size());
						memcpy_s(&t_rear.Speed, sizeof(t_rear.Speed), msg[2].data(), msg[2].size());
						memcpy_s(&t_steer.CurValue, sizeof(t_steer.CurValue), msg[3].data(), msg[3].size());
						memcpy_s(&t_steer.TarValue, sizeof(t_steer.TarValue), msg[4].data(), msg[4].size());
						memcpy_s(&t_steer.Speed, sizeof(t_steer.Speed), msg[5].data(), msg[5].size());
						m_currentState.UpdateMoveMotorState(t_rear, t_steer);
					}
					else if (topic == "STATE_CAMERA_MOTOR" && msg.size() == 6) {
						MotorStateType<float> t_cameraPitch;
						MotorStateType<float> t_cameraYaw;
						memcpy_s(&t_cameraPitch.CurValue, sizeof(t_cameraPitch.CurValue), msg[0].data(), msg[0].size());
						memcpy_s(&t_cameraPitch.TarValue, sizeof(t_cameraPitch.TarValue), msg[1].data(), msg[1].size());
						memcpy_s(&t_cameraPitch.Speed, sizeof(t_cameraPitch.Speed), msg[2].data(), msg[2].size());
						memcpy_s(&t_cameraYaw.CurValue, sizeof(t_cameraYaw.CurValue), msg[3].data(), msg[3].size());
						memcpy_s(&t_cameraYaw.TarValue, sizeof(t_cameraYaw.TarValue), msg[4].data(), msg[4].size());
						memcpy_s(&t_cameraYaw.Speed, sizeof(t_cameraYaw.Speed), msg[5].data(), msg[5].size());
						m_currentState.UpdateCameraMotorState(t_cameraPitch, t_cameraYaw);
					}
					else if (topic == "STATE_SENSOR" && msg.size() == 4) {
						double t_sonicSensor;
						vector<int> t_floorSensor(3, 0.0);
						memcpy_s(&t_sonicSensor, sizeof(t_sonicSensor), msg[0].data(), msg[0].size());
						memcpy_s(&t_floorSensor[0], sizeof(t_floorSensor[0]), msg[1].data(), msg[1].size());
						memcpy_s(&t_floorSensor[1], sizeof(t_floorSensor[1]), msg[2].data(), msg[2].size());
						memcpy_s(&t_floorSensor[2], sizeof(t_floorSensor[2]), msg[3].data(), msg[3].size());
						m_currentState.UpdateSensorState(t_sonicSensor, t_floorSensor);
					}
				}
			}
			catch (...) {}
		}

		subSocket.disconnect(m_pubAddress);
	}
	void Switcher::subscribeImageThreadFunc() {
		zmq::context_t context(2);
		zmq::socket_t subSocket(context, ZMQ_SUB);

		subSocket.set(zmq::sockopt::rcvhwm, 20);
		subSocket.set(zmq::sockopt::rcvtimeo, 10);
		subSocket.set(zmq::sockopt::subscribe, "STATE_CAMERA_SENSOR");
		subSocket.connect(m_pubAddress);

		while (!m_isStop) {
			try {
				zmq::multipart_t msg;
				if (msg.recv(subSocket) && msg.size() == 2) {
					vector<char> recvData(msg[1].size());
					memcpy(recvData.data(), msg[1].data(), msg[1].size());

					StateType stateInfo;
					m_currentState.Clone(stateInfo);

					Mat originImage = imdecode(recvData, IMREAD_ANYCOLOR);
					Mat stateImage = ImageFilter::ApplyStateInfo(originImage, stateInfo);
					Mat filterImage = ImageFilter::ApplyAdaptiveBrightness(originImage);
					m_currentState.UpdateCameraImage(originImage, stateImage, filterImage);
				}
			}
			catch (...) {}
		}

		subSocket.disconnect(m_pubAddress);
	}
	void Switcher::updateConnectionThreadFunc() {
		while (!m_isStop) {
			chrono::steady_clock::time_point start = chrono::steady_clock::now();

			shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
			msg->addstr("COMMAND_PICAR");
			msg->addstr("UPDATE_CONNECTION");
			m_queueCmdMutex.lock();
			m_queueCmd.push(msg);
			m_queueCmdMutex.unlock();


			this_thread::sleep_for(chrono::seconds(1) - (chrono::steady_clock::now() - start));
		}
	}

	void Switcher::GetOriginImage(ImageData& imgData) {
		imgData.Update(m_currentState.GetOriginImage());
	}
	void Switcher::GetStateImage(ImageData& imgData) {
		imgData.Update(m_currentState.GetStateImage());
	}
	void Switcher::GetFilterImage(ImageData& imgData) {
		imgData.Update(m_currentState.GetFilterImage());
	}

	void Switcher::TurnOff() {
		shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
		msg->addstr("COMMAND_PICAR");
		msg->addstr("TURN_OFF");
		m_queueCmdMutex.lock();
		m_queueCmd.push(msg);
		m_queueCmdMutex.unlock();
	}
	void Switcher::StopMove() {
		shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
		msg->addstr("COMMAND_MOVE_MOTOR");
		msg->addstr("REAR_MOTOR");
		msg->addstr("STOP");
		msg->addtyp<int>(0);
		m_queueCmdMutex.lock();
		m_queueCmd.push(msg);
		m_queueCmdMutex.unlock();
	}
	void Switcher::ChangeRearValue(int diff) {
		int targetVal = diff == 0 ? 0 : m_currentState.GetRear().CurValue + diff;
		shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
		msg->addstr("COMMAND_MOVE_MOTOR");
		msg->addstr("REAR_MOTOR");
		msg->addstr("VALUE");
		msg->addtyp<int>(targetVal);
		m_queueCmdMutex.lock();
		m_queueCmd.push(msg);
		m_queueCmdMutex.unlock();
	}
	void Switcher::ChangeSteerValue(float diff) {
		float targetVal = diff == 0.0 ? 0.0f : m_currentState.GetSteer().CurValue + diff;
		shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
		msg->addstr("COMMAND_MOVE_MOTOR");
		msg->addstr("STEER_MOTOR");
		msg->addstr("VALUE");
		msg->addtyp<float>(targetVal);
		m_queueCmdMutex.lock();
		m_queueCmd.push(msg);
		m_queueCmdMutex.unlock();
	}
	void Switcher::ChangeCameraPitchValue(float diff) {
		float targetVal = diff == 0.0 ? 0.0f : m_currentState.GetCameraPitch().CurValue + diff;
		shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
		msg->addstr("COMMAND_CAMERA_MOTOR");
		msg->addstr("PITCH_MOTOR");
		msg->addstr("VALUE");
		msg->addtyp<float>(targetVal);
		m_queueCmdMutex.lock();
		m_queueCmd.push(msg);
		m_queueCmdMutex.unlock();
	}
	void Switcher::ChangeCameraPitchYaw(float diff) {
		float targetVal = diff == 0.0 ? 0.0f : m_currentState.GetCameraYaw().CurValue + diff;
		shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
		msg->addstr("COMMAND_CAMERA_MOTOR");
		msg->addstr("YAW_MOTOR");
		msg->addstr("VALUE");
		msg->addtyp<float>(targetVal);
		m_queueCmdMutex.lock();
		m_queueCmd.push(msg);
		m_queueCmdMutex.unlock();
	}
}

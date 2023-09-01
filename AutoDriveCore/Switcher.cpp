#include "pch.h"
#include "Switcher.h"
#include "ImageFilter.h"

namespace AutoDriveCode {
	using namespace std;
	using namespace cv;


	shared_ptr<zmq::multipart_t> TryDequeue(queue<shared_ptr<zmq::multipart_t>>& queue, mutex& mutex) {
		shared_ptr<zmq::multipart_t> msg = NULL;

		mutex.lock();
		while (queue.size() > 0) {
			msg = queue.front();
			queue.pop();
		}
		mutex.unlock();

		return msg;
	}


	void Switcher::Init(string pubAddress, string subAddress) {
		m_isStop = false;
		m_pubAddress = pubAddress;
		m_subAddress = subAddress;

		m_subThread = thread(&Switcher::subscribeThreadFunc, this);
		m_pubThread = thread(&Switcher::publishThreadFunc, this);
		m_updateConnectionThread = thread(&Switcher::updateConnectionThreadFunc, this);
		m_updateStateThread = thread(&Switcher::updateStateThreadFunc, this);
		m_updateImageThread = thread(&Switcher::updateImageThreadFunc, this);
	}
	void Switcher::Release() {
		m_isStop = true;
		m_subThread.join();
		m_pubThread.join();
		m_updateConnectionThread.join();
		m_updateStateThread.join();
		m_updateImageThread.join();
	}

	void Switcher::subscribeThreadFunc() {
		zmq::context_t context(1);
		zmq::socket_t subSocket(context, ZMQ_SUB);

		subSocket.set(zmq::sockopt::rcvhwm, 20);
		subSocket.set(zmq::sockopt::rcvtimeo, 10);
		subSocket.set(zmq::sockopt::subscribe, "STATE_MOVE_MOTOR");
		subSocket.set(zmq::sockopt::subscribe, "STATE_CAMERA_MOTOR");
		subSocket.set(zmq::sockopt::subscribe, "STATE_SENSOR");
		subSocket.set(zmq::sockopt::subscribe, "STATE_CAMERA_SENSOR");
		subSocket.connect(m_pubAddress);

		while (!m_isStop) {
			try {
				shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
				if (!msg->recv(subSocket) || msg->size() == 0) continue;

				string topic = msg->at(0).to_string();
				if (topic == "STATE_MOVE_MOTOR" && msg->size() == 6) {
					m_queueStateMutex.lock();
					m_queueState.push(msg);
					m_queueStateMutex.unlock();
				}
				else if (topic == "STATE_CAMERA_MOTOR" && msg->size() == 6) {
					m_queueStateMutex.lock();
					m_queueState.push(msg);
					m_queueStateMutex.unlock();
				}
				else if (topic == "STATE_SENSOR" && msg->size() == 4) {
					m_queueStateMutex.lock();
					m_queueState.push(msg);
					m_queueStateMutex.unlock();
				}
				else if (topic == "STATE_CAMERA_SENSOR" && msg->size() == 2) {
					m_queueImageMutex.lock();
					m_queueImage.push(msg);
					m_queueImageMutex.unlock();
				}
			}
			catch (...) {}
		}

		subSocket.disconnect(m_pubAddress);
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
				shared_ptr<zmq::multipart_t> msg = TryDequeue(m_queueCmd, m_queueCmdMutex);
				if (msg == NULL) {
					this_thread::sleep_for(chrono::milliseconds(10));
					continue;
				}

				msg->send(pubSocket);
			}
			catch (...) {}
		}

		pubSocket.disconnect(m_subAddress);
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
	void Switcher::updateStateThreadFunc() {
		while (!m_isStop) {
			try {
				bool isContinue = false;
				shared_ptr<zmq::multipart_t> msg = TryDequeue(m_queueState, m_queueStateMutex);
				if (msg == NULL) {
					this_thread::sleep_for(chrono::milliseconds(10));
					continue;
				}

				string topic = msg->popstr();
				if (topic == "STATE_MOVE_MOTOR") {
					MotorStateType<int> t_rear;
					MotorStateType<float> t_steer;
					memcpy_s(&t_rear.CurValue, sizeof(t_rear.CurValue), msg->at(0).data(), msg->at(0).size());
					memcpy_s(&t_rear.TarValue, sizeof(t_rear.TarValue), msg->at(1).data(), msg->at(1).size());
					memcpy_s(&t_rear.Speed, sizeof(t_rear.Speed), msg->at(2).data(), msg->at(2).size());
					memcpy_s(&t_steer.CurValue, sizeof(t_steer.CurValue), msg->at(3).data(), msg->at(3).size());
					memcpy_s(&t_steer.TarValue, sizeof(t_steer.TarValue), msg->at(4).data(), msg->at(4).size());
					memcpy_s(&t_steer.Speed, sizeof(t_steer.Speed), msg->at(5).data(), msg->at(5).size());
					m_currentState.UpdateMoveMotorState(t_rear, t_steer);
				}
				else if (topic == "STATE_CAMERA_MOTOR") {
					MotorStateType<float> t_cameraPitch;
					MotorStateType<float> t_cameraYaw;
					memcpy_s(&t_cameraPitch.CurValue, sizeof(t_cameraPitch.CurValue), msg->at(0).data(), msg->at(0).size());
					memcpy_s(&t_cameraPitch.TarValue, sizeof(t_cameraPitch.TarValue), msg->at(1).data(), msg->at(1).size());
					memcpy_s(&t_cameraPitch.Speed, sizeof(t_cameraPitch.Speed), msg->at(2).data(), msg->at(2).size());
					memcpy_s(&t_cameraYaw.CurValue, sizeof(t_cameraYaw.CurValue), msg->at(3).data(), msg->at(3).size());
					memcpy_s(&t_cameraYaw.TarValue, sizeof(t_cameraYaw.TarValue), msg->at(4).data(), msg->at(4).size());
					memcpy_s(&t_cameraYaw.Speed, sizeof(t_cameraYaw.Speed), msg->at(5).data(), msg->at(5).size());
					m_currentState.UpdateCameraMotorState(t_cameraPitch, t_cameraYaw);
				}
				else if (topic == "STATE_SENSOR") {
					double t_sonicSensor;
					vector<int> t_floorSensor(3, 0.0);
					memcpy_s(&t_sonicSensor, sizeof(t_sonicSensor), msg->at(0).data(), msg->at(0).size());
					memcpy_s(&t_floorSensor[0], sizeof(t_floorSensor[0]), msg->at(1).data(), msg->at(1).size());
					memcpy_s(&t_floorSensor[1], sizeof(t_floorSensor[1]), msg->at(2).data(), msg->at(2).size());
					memcpy_s(&t_floorSensor[2], sizeof(t_floorSensor[2]), msg->at(3).data(), msg->at(3).size());
					m_currentState.UpdateSensorState(t_sonicSensor, t_floorSensor);
				}
			}
			catch (...) {}
		}
	}
	void Switcher::updateImageThreadFunc() {
		while (!m_isStop) {
			try {
				bool isContinue = false;
				shared_ptr<zmq::multipart_t> msg = TryDequeue(m_queueImage, m_queueImageMutex);
				if (msg == NULL) {
					this_thread::sleep_for(chrono::milliseconds(10));
					continue;
				}

				string topic = msg->popstr();
				zmq::message_t data = msg->pop();
				vector<char> recvData(data.size());
				memcpy(recvData.data(), data.data(), data.size());

				Mat originImage = imdecode(recvData, IMREAD_ANYCOLOR);
				Mat stateImage;
				Mat filterImage;
				thread stateImageThread(&Switcher::updateStateImageThreadFunc, this, &stateImage, &originImage);
				thread filterImageThread(&Switcher::updateFilterImageThreadFunc, this, &filterImage, &originImage);
				stateImageThread.join();
				filterImageThread.join();
				m_currentState.UpdateCameraImage(originImage, stateImage, filterImage);
			}
			catch (...) {}
		}
	}
	void Switcher::updateStateImageThreadFunc(Mat* stateImage, Mat* originImage) {
		StateType stateInfo;
		m_currentState.Clone(stateInfo);
		*stateImage = ImageFilter::ApplyStateInfo(*originImage, stateInfo);
	}
	void Switcher::updateFilterImageThreadFunc(Mat* filterImage, Mat* originImage) {
		*filterImage = ImageFilter::AdjustBrightness(*originImage);
	}

	void Switcher::GetOriginImage(ImageData& imgData) {
		Mat mat = m_currentState.GetOriginImage();
		imgData.Update(mat);
	}
	void Switcher::GetStateImage(ImageData& imgData) {
		Mat mat = m_currentState.GetStateImage();
		imgData.Update(mat);
	}
	void Switcher::GetFilterImage(ImageData& imgData) {
		Mat mat = m_currentState.GetFilterImage();
		imgData.Update(mat);
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

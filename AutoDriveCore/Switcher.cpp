#include "pch.h"
#include "Switcher.h"
#include "ImageFilter.h"

namespace AutoDriveCode {
	using namespace std;
	using namespace cv;


	shared_ptr<zmq::multipart_t> TryDequeue(queue<shared_ptr<zmq::multipart_t>>& queue, mutex& mutex, size_t hwm = 0) {
		shared_ptr<zmq::multipart_t> msg = NULL;

		mutex.lock();
		if (queue.size() > 0) {
			msg = queue.front();
			queue.pop();
			while (queue.size() > hwm) {
				msg = queue.front();
				queue.pop();
			}
		}
		mutex.unlock();

		return msg;
	}


	void Switcher::Init(string pubAddress, string subAddress, void* pcWndHandle) {
		m_isStop = false;
		m_pubAddress = pubAddress;
		m_subAddress = subAddress;

		m_caliData.Init();

		if (!pcWndHandle)
			m_viewer = NULL;
		else {
			vtkSmartPointer<vtkRenderer> renderer = vtkRenderer::New();
			vtkSmartPointer<vtkRenderWindowInteractor> interActor = vtkRenderWindowInteractor::New();
			vtkSmartPointer<vtkRenderWindow> window = vtkRenderWindow::New();
			interActor->SetRenderWindow(window);
			renderer->SetBackground(0, 0, 0);
			window->AddRenderer(renderer);
			window->SetParentId(pcWndHandle);
			window->DoubleBufferOn();

			pcl::visualization::PCLVisualizer::Ptr t_viewer(new pcl::visualization::PCLVisualizer(renderer, window, "Point Cloud Viewer"));
			m_viewer = t_viewer;
			m_viewer->initCameraParameters();
			m_viewer->setCameraPosition(0, 0, 0, 0, 100, 0, 0, 0, 1);
			//m_viewer->setCameraPosition(0, 0, 100, 0, 0, 0, 0, 1, 0);
			m_viewer->setCameraClipDistances(1, 300);
			m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10);
			m_viewer->addCube(-5, 5, -10, 0, -5, 0, 1.0, 1.0, 1.0, "Lidar Object");
			m_viewer->addCube(-5, 5, -10, 0, -5, 0, 1.0, 1.0, 1.0, "Car Object");
		}

		m_subThread = make_shared<thread>(&Switcher::subscribeThreadFunc, this);
		m_pubThread = make_shared<thread>(&Switcher::publishThreadFunc, this);
		m_updateConnectionThread = make_shared<thread>(&Switcher::updateConnectionThreadFunc, this);
		m_updateStateThread = make_shared<thread>(&Switcher::updateStateThreadFunc, this);
		m_updateImageThread = make_shared<thread>(&Switcher::updateImageThreadFunc, this);
		m_updateLidarThread = make_shared<thread>(&Switcher::updateLidarThreadFunc, this);
		m_updateConclusionThread = make_shared<thread>(&Switcher::updateConclusionThreadFunc, this);
	}
	void Switcher::Release() {
		m_isStop = true;
		if (m_subThread) m_subThread->join();
		if (m_pubThread) m_pubThread->join();
		if (m_updateConnectionThread) m_updateConnectionThread->join();
		if (m_updateStateThread) m_updateStateThread->join();
		if (m_updateImageThread) m_updateImageThread->join();
		if (m_updateLidarThread) m_updateLidarThread->join();
		if (m_updateConclusionThread) m_updateConclusionThread->join();
	}

	void Switcher::subscribeThreadFunc() {
		zmq::context_t context(1);
		zmq::socket_t subSocket(context, ZMQ_SUB);

		subSocket.set(zmq::sockopt::rcvhwm, 20);
		subSocket.set(zmq::sockopt::rcvtimeo, 10);
		subSocket.set(zmq::sockopt::subscribe, "STATE_MOVE_MOTOR");
		subSocket.set(zmq::sockopt::subscribe, "STATE_CAMERA_MOTOR");
		subSocket.set(zmq::sockopt::subscribe, "STATE_SENSOR");
		subSocket.set(zmq::sockopt::subscribe, "STATE_LCD_DISPLAY");
		subSocket.set(zmq::sockopt::subscribe, "STATE_LIDAR_SENSOR");
		subSocket.set(zmq::sockopt::subscribe, "STATE_CAMERA_SENSOR");
		subSocket.connect(m_pubAddress);

		while (!m_isStop) {
			try {
				shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
				if (!msg->recv(subSocket) || msg->size() == 0) continue;

				string topic = msg->at(0).to_string();
				if (topic == "STATE_MOVE_MOTOR" && msg->size() == 7) {
					m_queueStateMutex.lock();
					m_queueState.push(msg);
					m_queueStateMutex.unlock();
				}
				else if (topic == "STATE_CAMERA_MOTOR" && msg->size() == 7) {
					m_queueStateMutex.lock();
					m_queueState.push(msg);
					m_queueStateMutex.unlock();
				}
				else if (topic == "STATE_SENSOR" && msg->size() == 5) {
					m_queueStateMutex.lock();
					m_queueState.push(msg);
					m_queueStateMutex.unlock();
				}
				else if (topic == "STATE_LCD_DISPLAY" && msg->size() == 3) {
					m_queueStateMutex.lock();
					m_queueState.push(msg);
					m_queueStateMutex.unlock();
				}
				else if (topic == "STATE_LIDAR_SENSOR" && msg->size() % 3 == 2) {
					m_queueLidarMutex.lock();
					m_queueLidar.push(msg);
					m_queueLidarMutex.unlock();
				}
				else if (topic == "STATE_CAMERA_SENSOR" && msg->size() == 2) {
					m_queueLidarMutex.lock();
					m_queueImage.push(msg);
					m_queueLidarMutex.unlock();
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
		pubSocket.connect(m_subAddress);

		while (!m_isStop) {
			try {
				bool isContinue = false;
				shared_ptr<zmq::multipart_t> msg = TryDequeue(m_queueCmd, m_queueCmdMutex, 4);
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
			ClockType::time_point start = ClockType::now();

			shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
			msg->addstr("COMMAND_PICAR");
			msg->addstr("UPDATE_CONNECTION");
			m_queueCmdMutex.lock();
			m_queueCmd.push(msg);
			m_queueCmdMutex.unlock();

			chrono::milliseconds sleepTime = chrono::duration_cast<chrono::milliseconds>(chrono::seconds(1) - (ClockType::now() - start));
			while (!m_isStop && sleepTime.count() > 0) {
				chrono::milliseconds t_sleepTime =
					sleepTime.count() > 100 ?
					chrono::milliseconds(100) :
					sleepTime;
				this_thread::sleep_for(t_sleepTime);
				sleepTime -= t_sleepTime;
			}
		}
	}
	void Switcher::updateStateThreadFunc() {
		while (!m_isStop) {
			try {
				bool isContinue = false;
				shared_ptr<zmq::multipart_t> msg = TryDequeue(m_queueState, m_queueStateMutex, 4);
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
					m_currentMachineState.UpdateMoveMotorState(t_rear, t_steer);
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
					m_currentMachineState.UpdateCameraMotorState(t_cameraPitch, t_cameraYaw);
				}
				else if (topic == "STATE_SENSOR") {
					double t_sonicSensor;
					vector<int> t_floorSensor(3, 0.0);
					memcpy_s(&t_sonicSensor, sizeof(t_sonicSensor), msg->at(0).data(), msg->at(0).size());
					memcpy_s(&t_floorSensor[0], sizeof(t_floorSensor[0]), msg->at(1).data(), msg->at(1).size());
					memcpy_s(&t_floorSensor[1], sizeof(t_floorSensor[1]), msg->at(2).data(), msg->at(2).size());
					memcpy_s(&t_floorSensor[2], sizeof(t_floorSensor[2]), msg->at(3).data(), msg->at(3).size());
					m_currentMachineState.UpdateSensorState(t_sonicSensor, t_floorSensor);
				}
				else if (topic == "STATE_LCD_DISPLAY") {
					double t_temp;
					int t_state;
					memcpy_s(&t_temp, sizeof(t_temp), msg->at(0).data(), msg->at(0).size());
					memcpy_s(&t_state, sizeof(t_state), msg->at(1).data(), msg->at(1).size());
					m_currentMachineState.UpdateLcdState(t_temp, t_state);
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

				Mat img = imdecode(recvData, IMREAD_ANYCOLOR);
				MachineStateType stateInfo;
				m_currentMachineState.Clone(stateInfo);
				Mat t_stateImg = ImageFilter::DrawStateInfoFilter(img, stateInfo);
				m_currentCameraImage.Set(img, stateInfo.GetCameraPitch().CurValue, stateInfo.GetCameraYaw().CurValue);
				m_stateImageMutex.lock();
				m_stateImage = t_stateImg;
				m_stateImageMutex.unlock();
			}
			catch (...) {}
		}
	}
	void Switcher::updateLidarThreadFunc() {
		while (!m_isStop) {
			try {
				bool isContinue = false;
				shared_ptr<zmq::multipart_t> msg = TryDequeue(m_queueLidar, m_queueLidarMutex);
				if (msg == NULL) {
					this_thread::sleep_for(chrono::milliseconds(10));
					continue;
				}

				string topic = msg->popstr();
				int len = msg->poptyp<int>();
				if (len * 3 != msg->size())
					continue;

				PTCPtr pcPtr(new PTCType());
				for (int i = 0; i < len; i++) {
					float degree = msg->poptyp<float>();
					float radian = DEGREE_TO_RADIAN(degree);
					float distance = msg->poptyp<float>() * 0.1;
					float intensity = msg->poptyp<float>();
					PTType t_pt;
					t_pt.x = distance * sin(radian);
					t_pt.y = distance * cos(radian);
					t_pt.z = 6.f;
					pcPtr->push_back(t_pt);
				}

				m_targetLidarPoint.Set(pcPtr);
			}
			catch (...) {}
		}
	}
	void Switcher::updateConclusionThreadFunc() {
		ClockType::time_point last_image_time, last_lidar_time;
		ClockType::time_point cur_image_time, cur_lidar_time;
		last_image_time = last_lidar_time = ClockType::time_point::min();

		Mat cur_Img;
		float pitch_degree, yaw_degree;
		PTCPtr cur_lidar;

		PCL_NEW(PTLCType, dst_image_pc);
		PCL_NEW(PTCType, dst_lidar_pc);

		while (!m_isStop) {
			auto start = ClockType::now();

			m_targetCameraImage.Get(cur_image_time, cur_Img, pitch_degree, yaw_degree);
			m_targetLidarPoint.Get(cur_lidar_time, cur_lidar);

			if (!cur_Img.empty() && last_image_time != cur_image_time) {
				last_image_time = cur_image_time;
				dst_image_pc = ImageFilter::ConvertImageToPointCloud(
					cur_Img,
					m_caliData.Margin,
					m_caliData.PointCloudSize,
					m_caliData.PointOffsets,
					pitch_degree,
					yaw_degree
				);
			}
			if (cur_lidar && last_lidar_time != cur_lidar_time) {
				last_lidar_time = cur_lidar_time;
				dst_lidar_pc = cur_lidar;
			}

			if (m_viewer && !m_viewer->wasStopped()) {
				if (!m_viewer->updatePointCloud(dst_image_pc, "IMAGE_POINT"))
					m_viewer->addPointCloud(dst_image_pc, "IMAGE_POINT");
				if (!m_viewer->updatePointCloud(dst_lidar_pc, "LIDAR_POINT"))
					m_viewer->addPointCloud(dst_lidar_pc, "LIDAR_POINT");
				m_viewer->spinOnce();
			}

			auto updateTime = chrono::duration_cast<chrono::milliseconds>(ClockType::now() - start).count();
			if (updateTime < 66)
				this_thread::sleep_for(chrono::milliseconds(66 - updateTime));
		}
	}

	void Switcher::ExecuteEventCameraPointCloudCapture() {
		m_targetCameraImage.Set(m_currentCameraImage);
	}

	void Switcher::GetStateImage(ImageData& imgData) {
		m_stateImageMutex.lock();
		imgData.Update(m_stateImage);
		m_stateImageMutex.unlock();
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
	void Switcher::SetRearValue(CMD_VALUE_TYPE type, int value) {
		shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
		msg->addstr("COMMAND_MOVE_MOTOR");
		msg->addstr("REAR_MOTOR");
		switch (type) {
		case CMD_VALUE_TYPE_TARGET:	msg->addstr("VALUE");	break;
		case CMD_VALUE_TYPE_SPEED:	msg->addstr("SPEED");	break;
		default: return;
		}
		msg->addtyp<int>(value);
		m_queueCmdMutex.lock();
		m_queueCmd.push(msg);
		m_queueCmdMutex.unlock();
	}
	void Switcher::SetSteerValue(CMD_VALUE_TYPE type, float value) {
		shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
		msg->addstr("COMMAND_MOVE_MOTOR");
		msg->addstr("STEER_MOTOR");
		switch (type) {
		case CMD_VALUE_TYPE_TARGET:	msg->addstr("VALUE");	break;
		case CMD_VALUE_TYPE_SPEED:	msg->addstr("SPEED");	break;
		default: return;
		}
		msg->addtyp<float>(value);
		m_queueCmdMutex.lock();
		m_queueCmd.push(msg);
		m_queueCmdMutex.unlock();
	}
	void Switcher::SetCameraPitchValue(CMD_VALUE_TYPE type, float value) {
		shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
		msg->addstr("COMMAND_CAMERA_MOTOR");
		msg->addstr("PITCH_MOTOR");
		switch (type) {
		case CMD_VALUE_TYPE_TARGET:	msg->addstr("VALUE");	break;
		case CMD_VALUE_TYPE_SPEED:	msg->addstr("SPEED");	break;
		default: return;
		}
		msg->addtyp<float>(value);
		m_queueCmdMutex.lock();
		m_queueCmd.push(msg);
		m_queueCmdMutex.unlock();
	}
	void Switcher::SetCameraYawValue(CMD_VALUE_TYPE type, float value) {
		shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
		msg->addstr("COMMAND_CAMERA_MOTOR");
		msg->addstr("YAW_MOTOR");
		switch (type) {
		case CMD_VALUE_TYPE_TARGET:	msg->addstr("VALUE");	break;
		case CMD_VALUE_TYPE_SPEED:	msg->addstr("SPEED");	break;
		default: return;
		}
		msg->addtyp<float>(value);
		m_queueCmdMutex.lock();
		m_queueCmd.push(msg);
		m_queueCmdMutex.unlock();
	}

	void Switcher::ChangeRearValue(int diff) {
		int targetVal =
			diff == 0 ?
			0 :
			m_currentMachineState.GetRear().CurValue + diff;
		SetRearValue(CMD_VALUE_TYPE_TARGET, targetVal);
	}
	void Switcher::ChangeSteerValue(float diff) {
		float targetVal =
			diff == 0.0 ?
			0.0f :
			m_currentMachineState.GetSteer().CurValue + diff;
		SetSteerValue(CMD_VALUE_TYPE_TARGET, targetVal);
	}
	void Switcher::ChangeCameraPitchValue(float diff) {
		float targetVal =
			diff == 0.0 ?
			0.0f :
			m_currentMachineState.GetCameraPitch().CurValue + diff;
		SetCameraPitchValue(CMD_VALUE_TYPE_TARGET, targetVal);
	}
	void Switcher::ChangeCameraYawValue(float diff) {
		float targetVal =
			diff == 0.0 ?
			0.0f :
			m_currentMachineState.GetCameraYaw().CurValue + diff;
		SetCameraYawValue(CMD_VALUE_TYPE_TARGET, targetVal);
	}
}


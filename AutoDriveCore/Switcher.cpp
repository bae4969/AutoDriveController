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


	void Switcher::Init(string pubAddress, string subAddress) {
		m_isStop = false;
		m_pubAddress = pubAddress;
		m_subAddress = subAddress;
		m_cameraCaliData.Init();

		m_subThread = thread(&Switcher::subscribeThreadFunc, this);
		m_pubThread = thread(&Switcher::publishThreadFunc, this);
		m_updateConnectionThread = thread(&Switcher::updateConnectionThreadFunc, this);
		m_updateStateThread = thread(&Switcher::updateStateThreadFunc, this);
		m_updateImageThread = thread(&Switcher::updateImageThreadFunc, this);

		m_isBusy = false;
	}
	void Switcher::Release() {
		m_isStop = true;
		m_subThread.join();
		m_pubThread.join();
		m_updateConnectionThread.join();
		m_updateStateThread.join();
		m_updateImageThread.join();
		m_updateViewerThread.join();
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
			chrono::steady_clock::time_point start = chrono::steady_clock::now();

			shared_ptr<zmq::multipart_t> msg(new zmq::multipart_t);
			msg->addstr("COMMAND_PICAR");
			msg->addstr("UPDATE_CONNECTION");
			m_queueCmdMutex.lock();
			m_queueCmd.push(msg);
			m_queueCmdMutex.unlock();

			chrono::milliseconds sleepTime = chrono::duration_cast<chrono::milliseconds>(chrono::seconds(1) - (chrono::steady_clock::now() - start));
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

				thread th(&Switcher::updatePipelineThreadFunc, this, originImage);
				th.detach();
			}
			catch (...) {}
		}
	}
	void Switcher::updateViewerThreadFunc() {
		float startPitch = -20.f;
		float startYaw = -40.f;
		float diffPitch = 20.f;
		float diffYaw = 20.f;
		int stepPitch = 4;
		int stepYaw = 5;

		vector<pair<float, float>> table;
		for (int yIdx = 0; yIdx < stepPitch; yIdx++) {
			for (int xIdx = 0; xIdx < stepYaw; xIdx++) {
				int totIdx = yIdx * stepPitch + xIdx;

				float pitchDegree = diffPitch * (yIdx - 1);
				float yawDegree =
					yIdx % 2 ?
					diffYaw * (2 - xIdx) :
					diffYaw * (xIdx - 2);

				table.push_back(make_pair(pitchDegree, yawDegree));
			}
		}

		SetCameraPitchValue(CMD_VALUE_TYPE_TARGET, startPitch);
		SetCameraPitchValue(CMD_VALUE_TYPE_SPEED, 200);
		SetCameraYawValue(CMD_VALUE_TYPE_TARGET, startYaw);
		SetCameraYawValue(CMD_VALUE_TYPE_SPEED, 200);
		this_thread::sleep_for(chrono::milliseconds(3000));

		while (!m_isStop && !m_viewer->wasStopped())
		{
			for (int idx = 0; !m_isStop && idx < stepPitch * stepYaw; idx++) {
				pair<float, float> t_value = table[idx];

				string pcName = std::format("Point Cloud {:d}", idx);
				float pitchDegree = t_value.first;
				float yawDegree = t_value.second;

				SetCameraPitchValue(CMD_VALUE_TYPE_TARGET, pitchDegree);
				SetCameraYawValue(CMD_VALUE_TYPE_TARGET, yawDegree);
				this_thread::sleep_for(chrono::milliseconds(500));

				PTLCPtr pcPtr = m_currentCameraState.GetPointCloud();
				if (pcPtr && !m_viewer->updatePointCloud(pcPtr, pcName))
				{
					m_viewer->addPointCloud(pcPtr, pcName);
				}

				m_viewer->spinOnce();
				this_thread::sleep_for(chrono::milliseconds(66));
			}

			for (int idx = stepPitch * stepYaw - 2; !m_isStop && idx > 0; idx--) {
				pair<float, float> t_value = table[idx];

				string pcName = std::format("Point Cloud {:d}", idx);
				float pitchDegree = t_value.first;
				float yawDegree = t_value.second;

				SetCameraPitchValue(CMD_VALUE_TYPE_TARGET, pitchDegree);
				SetCameraYawValue(CMD_VALUE_TYPE_TARGET, yawDegree);
				this_thread::sleep_for(chrono::milliseconds(500));

				PTLCPtr pcPtr = m_currentCameraState.GetPointCloud();
				if (pcPtr && !m_viewer->updatePointCloud(pcPtr, pcName))
				{
					m_viewer->addPointCloud(pcPtr, pcName);
				}

				m_viewer->spinOnce();
				this_thread::sleep_for(chrono::milliseconds(66));
			}
		}
	}
	void Switcher::updatePipelineThreadFunc(Mat originImage) {
		m_currentCameraState.UpdateOriginImage(originImage);
		if (m_isBusy) return;
		m_currentMachineState.UpdateFPS();

		m_pipelineMutex[0].lock();
		m_isBusy = true;

		MachineStateType stateInfo;
		m_currentMachineState.Clone(stateInfo);
		Mat stateImage = ImageFilter::DrawStateInfoFilter(originImage, stateInfo);
		m_currentCameraState.UpdateStateImage(stateImage);

		m_isBusy = false;
		m_pipelineMutex[1].lock();
		m_pipelineMutex[0].unlock();

		Mat filterImage = ImageFilter::CalibrationCameraFilter(originImage, m_cameraCaliData);
		m_currentCameraState.UpdateFilterImage(filterImage);

		m_pipelineMutex[2].lock();
		m_pipelineMutex[1].unlock();

		PTLCPtr pcPtr = ImageFilter::ConvertImageToPointCloud(originImage, m_cameraCaliData, stateInfo);
		m_currentCameraState.UpdatePointCloud(pcPtr);

		m_pipelineMutex[2].unlock();
	}

	void Switcher::GetOriginImage(ImageData& imgData) {
		Mat mat = m_currentCameraState.GetOriginImage();
		imgData.Update(mat);
	}
	void Switcher::GetStateImage(ImageData& imgData) {
		Mat mat = m_currentCameraState.GetStateImage();
		imgData.Update(mat);
	}
	void Switcher::GetFilterImage(ImageData& imgData) {
		Mat mat = m_currentCameraState.GetFilterImage();
		imgData.Update(mat);
	}

	void Switcher::SetPointCloudViwerWindow(void* handle) {
		vtkSmartPointer<vtkRenderer> renderer = vtkRenderer::New();
		vtkSmartPointer<vtkRenderWindowInteractor> interActor = vtkRenderWindowInteractor::New();
		vtkSmartPointer<vtkRenderWindow> window = vtkRenderWindow::New();
		interActor->SetRenderWindow(window);
		renderer->SetBackground(0, 0, 0);
		window->AddRenderer(renderer);
		window->SetParentId(handle);
		window->DoubleBufferOn();

		pcl::visualization::PCLVisualizer::Ptr t_viewer(new pcl::visualization::PCLVisualizer(renderer, window, "Point Cloud Viewer"));
		m_viewer = t_viewer;
		m_viewer->initCameraParameters();
		m_viewer->setCameraPosition(0, 0, 0, 0, 100, 0, 0, 0, 1);
		//m_viewer->setCameraPosition(0, 0, 100, 0, 0, 0, 0, 1, 0);
		m_viewer->setCameraClipDistances(1, 300);
		m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10);
		m_viewer->addCube(-5, 5, -10, 0, -5, 0, 1.0, 1.0, 1.0, "Car Object");

		m_updateViewerThread = thread(&Switcher::updateViewerThreadFunc, this);
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
		SetRearValue(CMD_VALUE_TYPE_TARGET, diff);
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


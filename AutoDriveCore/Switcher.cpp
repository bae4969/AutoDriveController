#include "pch.h"
#include "Switcher.h"
#include "MachineDebugImageFilter.h"
#include "ImageToPointCloudFilter.h"

namespace AutoDriveCode {
	using namespace std;
	using namespace cv;


	shared_ptr<zmq::multipart_t> TryDequeueMassage(queue<shared_ptr<zmq::multipart_t>>& queue, mutex& mutex, size_t hwm = 0) {
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

		if (!pcWndHandle)
			m_viewer = NULL;
		else {
			vtkSmartPointer<vtkRenderer> renderer = vtkRenderer::New();
			vtkSmartPointer<vtkRenderWindowInteractor> interActor = vtkRenderWindowInteractor::New();
			vtkSmartPointer<vtkRenderWindow> window = vtkRenderWindow::New();
			interActor->SetRenderWindow(window);
			renderer->SetBackground(0.25, 0.25, 0.25);
			window->AddRenderer(renderer);
			window->SetParentId(pcWndHandle);
			window->DoubleBufferOn();

			pcl::visualization::PCLVisualizer::Ptr t_viewer(new pcl::visualization::PCLVisualizer(renderer, window, "Point Cloud Viewer"));
			m_viewer = t_viewer;
			m_viewer->initCameraParameters();
			//m_viewer->setCameraPosition(0, 0, 0, 0, 100, 0, 0, 0, 1);
			m_viewer->setCameraPosition(0, -1000, 500, 0, 0, 0, 0, 0.666, 0.333);
			m_viewer->setCameraClipDistances(1, 3000);
			m_viewer->addCube(-30, 30, -35, 35, -30, 60, 1.0, 1.0, 1.0, "Core Object");
			m_viewer->addCube(-65, 65, -50, 160, -110, -30, 1.0, 1.0, 1.0, "Body Object");
			m_viewer->addCube(-20, 20, 110, 150, -20, 20, 1.0, 1.0, 1.0, "Camera Object");
			m_viewer->addCube(-15, 15, -15, 15, 60, 90, 1.0, 1.0, 1.0, "Lidar Object");
			ExecuteEventResizeVisualizer();
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
				shared_ptr<zmq::multipart_t> msg = TryDequeueMassage(m_queueCmd, m_queueCmdMutex, 4);
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
				shared_ptr<zmq::multipart_t> msg = TryDequeueMassage(m_queueState, m_queueStateMutex, 4);
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
					m_lastMachineState.UpdateMoveMotorState(t_rear, t_steer);
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
					m_lastMachineState.UpdateCameraMotorState(t_cameraPitch, t_cameraYaw);
				}
				else if (topic == "STATE_SENSOR") {
					double t_sonicSensor;
					vector<int> t_floorSensor(3, 0.0);
					memcpy_s(&t_sonicSensor, sizeof(t_sonicSensor), msg->at(0).data(), msg->at(0).size());
					memcpy_s(&t_floorSensor[0], sizeof(t_floorSensor[0]), msg->at(1).data(), msg->at(1).size());
					memcpy_s(&t_floorSensor[1], sizeof(t_floorSensor[1]), msg->at(2).data(), msg->at(2).size());
					memcpy_s(&t_floorSensor[2], sizeof(t_floorSensor[2]), msg->at(3).data(), msg->at(3).size());
					m_lastMachineState.UpdateSensorState(t_sonicSensor, t_floorSensor);
				}
				else if (topic == "STATE_LCD_DISPLAY") {
					double t_temp;
					int t_state;
					memcpy_s(&t_temp, sizeof(t_temp), msg->at(0).data(), msg->at(0).size());
					memcpy_s(&t_state, sizeof(t_state), msg->at(1).data(), msg->at(1).size());
					m_lastMachineState.UpdateLcdState(t_temp, t_state);
				}
			}
			catch (...) {}
		}
	}
	void Switcher::updateImageThreadFunc() {
		Modules::MachineDebugImageFilterType machineDebugImageFilter;
		machineDebugImageFilter.SetInplace(false);

		Modules::ImageToPointCloudFilterType imageToPointCloudFilter;
		imageToPointCloudFilter.SetMargin(CAMERA_EDGE_BAD_PIXEL_MARGIN);
		imageToPointCloudFilter.SetScale(IMAGE_TO_POINT_SIZE_RATE);
		imageToPointCloudFilter.SetDefaultDistance(IMAGE_POINT_DEFAULT_DISTANCE);
		imageToPointCloudFilter.SetOffsetDistance(CAMERA_TO_YAW_AXIS_Y_OFFSET);
		imageToPointCloudFilter.SetWidthMPPInOneMmDistance(CAMERA_WIDTH_MPP_IN_ONE_MM_DISTANCE);
		imageToPointCloudFilter.SetHeightMPPInOneMmDistance(CAMERA_HEIGHT_MPP_IN_ONE_MM_DISTANCE);
		imageToPointCloudFilter.SetRollDegree(CAMERA_ROLL_DEGREE);
		imageToPointCloudFilter.SetCenterY(YAW_AXIS_TO_CENTER_Y_OFFSET);

		while (!m_isStop) {
			try {
				bool isContinue = false;
				shared_ptr<zmq::multipart_t> msg = TryDequeueMassage(m_queueImage, m_queueImageMutex);
				if (msg == NULL) {
					this_thread::sleep_for(chrono::milliseconds(10));
					continue;
				}

				string topic = msg->popstr();
				zmq::message_t data = msg->pop();
				vector<char> recvData(data.size());
				memcpy(recvData.data(), data.data(), data.size());

				Mat img = imdecode(recvData, IMREAD_ANYCOLOR);
				m_lastMachineState.UpdateFPS();

				machineDebugImageFilter.SetWithMachineState(m_lastMachineState);
				machineDebugImageFilter.SetInputImage(img);
				machineDebugImageFilter.Update();

				imageToPointCloudFilter.SetInputImage(img);
				imageToPointCloudFilter.SetPitchDegree(machineDebugImageFilter.GetCameraPitchDegree());
				imageToPointCloudFilter.SetYawDegree(machineDebugImageFilter.GetCameraYawDegree());
				imageToPointCloudFilter.Update();

				m_lastStateImage.Set(machineDebugImageFilter.GetOutputImage());
				m_lastImagePc.Set(imageToPointCloudFilter.GetOutputPointCloud());
			}
			catch (...) {}
		}
	}
	void Switcher::updateLidarThreadFunc() {
		while (!m_isStop) {
			try {
				bool isContinue = false;
				shared_ptr<zmq::multipart_t> msg = TryDequeueMassage(m_queueLidar, m_queueLidarMutex);
				if (msg == NULL) {
					this_thread::sleep_for(chrono::milliseconds(10));
					continue;
				}

				string topic = msg->popstr();
				int len = msg->poptyp<int>();
				if (len * 3 != msg->size())
					continue;

				PTNCPtr pcPtr(new PTNCType());
				for (int i = 0; i < len; i++) {
					float degree = msg->poptyp<float>() + LIDAR_DEGREE_OFFSET;
					float radian = DEGREE_TO_RADIAN(degree);
					float distance = msg->poptyp<float>();
					float intensity = msg->poptyp<float>();
					PTNType t_pt;
					t_pt.normal_x = sin(radian);
					t_pt.normal_y = cos(radian);
					t_pt.normal_z = 0.0;
					t_pt.curvature = 0.0;
					t_pt.x = distance * t_pt.normal_x;
					t_pt.y = distance * t_pt.normal_y;
					t_pt.z = LIDAR_HEIGHT_OFFSET;
					pcPtr->push_back(t_pt);
				}

				TimePointType tp;
				PTNCPtr lastPc;
				m_lastLidarPc.Get(tp, lastPc);
				ExecuteEventRegistarteRidarPointCloud();
				m_lastLidarPc.Set(pcPtr);
			}
			catch (...) {}
		}
	}
	void Switcher::updateConclusionThreadFunc() {
		TimePointType last_image_time, last_lidar_time;
		TimePointType cur_image_time, cur_lidar_time;
		last_image_time = last_lidar_time = TimePointType::min();

		PTLNCPtr src_Image;
		PTNCPtr src_lidar;

		Eigen::Matrix4f delta_trans;

		PCL_NEW(PTLNCType, dst_image);
		PCL_NEW(PTNCType, dst_lidar);

		while (!m_isStop) {
			auto start = ClockType::now();

			m_curCameraPc.Get(cur_image_time, src_Image);
			m_curLidarPc.Get(cur_lidar_time, src_lidar);

			delta_trans.setIdentity();
			if (src_lidar && last_lidar_time != cur_lidar_time) {
				last_lidar_time = cur_lidar_time;
				dst_lidar = src_lidar;
			}
			if (src_Image && last_image_time != cur_image_time) {
				last_image_time = cur_image_time;
				dst_image = src_Image;
			}

			if (m_viewer && !m_viewer->wasStopped()) {
				PCL_NEW(PTLCType, viewer_image_pc);
				PCL_NEW(PTCType, viewer_lidar_pc);
				copyPointCloud<PTLNType, PTLType>(*dst_image, *viewer_image_pc);
				copyPointCloud<PTNType, PTType>(*dst_lidar, *viewer_lidar_pc);
				if (!m_viewer->updatePointCloud(viewer_image_pc, "IMAGE_POINT")) {
					m_viewer->addPointCloud(viewer_image_pc, "IMAGE_POINT");
					m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "IMAGE_POINT");
				}
				if (!m_viewer->updatePointCloud(viewer_lidar_pc, "LIDAR_POINT")) {
					m_viewer->addPointCloud(viewer_lidar_pc, "LIDAR_POINT");
					m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "LIDAR_POINT");
				}
			}

			auto updateTime = chrono::duration_cast<chrono::milliseconds>(ClockType::now() - start).count();
			if (m_viewer && !m_viewer->wasStopped())
				m_viewer->spinOnce(updateTime);
			if (updateTime < 66)
				this_thread::sleep_for(chrono::milliseconds(66 - updateTime));
		}
	}

	void Switcher::GetStateImage(ImageData& imgData) {
		TimePointType tp;
		Mat img;
		m_lastStateImage.Get(tp, img);
		imgData.Update(img);
	}
	void Switcher::ExecuteEventResizeVisualizer() {
		if (!m_viewer) return;

		auto renderWindow = m_viewer->getRenderWindow();
		RECT rect;
		GetWindowRect((HWND)renderWindow->GetGenericParentId(), &rect);
		renderWindow->SetSize(rect.right - rect.left, rect.bottom - rect.top);
	}
	void Switcher::ExecuteEventPushImagePointCloud() {
		TimePointType tp;
		PTLNCPtr pc;
		m_lastImagePc.Get(tp, pc);
		if (!pc || pc->size() == 0) return;

		m_imagePcBuffer.push(pc);
	}
	void Switcher::ExecuteEventPopAllImagePointCloud() {
		using SearchType = pcl::search::KdTree<PTLNType>;
		using FeatureEstimationType = pcl::PFHRGBEstimation<PTLNType, PTLNType>;
		using CorrespondenceEstimationType = pcl::registration::CorrespondenceEstimation<RgbFType, RgbFType>;
		using CorrespondenceRejection1Type = pcl::registration::CorrespondenceRejectorFeatures;
		using CorrespondenceRejection2Type = pcl::registration::CorrespondenceRejectorSampleConsensus<PTLNType>;
		using IcpFilterType = pcl::IterativeClosestPointWithNormals<PTLNType, PTLNType>;
		using TransformEstimationType = pcl::registration::TransformationEstimationSVD<PTLNType, PTLNType>;
		using DownSampleFilterType = pcl::VoxelGrid<PTLNType>;

		double rate = CAMERA_WIDTH_MPP_IN_ONE_MM_DISTANCE * IMAGE_POINT_DEFAULT_DISTANCE / IMAGE_TO_POINT_SIZE_RATE;
		PCL_NEW(SearchType, searchMethod);
		PCL_NEW(FeatureEstimationType, featureEstimation);
		PCL_NEW(CorrespondenceEstimationType, corrEstimation);
		PCL_NEW(CorrespondenceRejection1Type, corrRejection1);
		PCL_NEW(CorrespondenceRejection2Type, corrRejection2);
		PCL_NEW(IcpFilterType, icpFilter);
		PCL_NEW(TransformEstimationType, transEstimation);
		PCL_NEW(DownSampleFilterType, downSampleFilter);

		featureEstimation->setSearchMethod(searchMethod);
		featureEstimation->setKSearch(10);
		icpFilter->setMaxCorrespondenceDistance(rate * 5);
		downSampleFilter->setLeafSize(rate * 2, rate * 2, rate * 2);

		PCL_NEW(PTLNCType, dst);
		if (m_imagePcBuffer.size() == 1) {
			dst = m_imagePcBuffer.front();
			m_imagePcBuffer.pop();
		}
		else if (m_imagePcBuffer.size() > 1) {
			PCL_NEW(PTLNCType, src1);
			PCL_NEW(PTLNCType, src2);
			PCL_NEW(PTLNCType, src3);
			PCL_NEW(RgbFCType, feature1);
			PCL_NEW(RgbFCType, feature2);
			PCL_NEW(RgbFCType, feature3);
			pcl::CorrespondencesPtr allCorr(new pcl::Correspondences);
			pcl::CorrespondencesPtr featureCorr(new pcl::Correspondences);
			pcl::CorrespondencesPtr sampleCorr(new pcl::Correspondences);

			src1 = m_imagePcBuffer.front();
			m_imagePcBuffer.pop();
			featureEstimation->setInputCloud(src1);
			featureEstimation->setInputNormals(src1);
			featureEstimation->compute(*feature1);

			while (m_imagePcBuffer.size() > 0) {
				src2 = m_imagePcBuffer.front();
				m_imagePcBuffer.pop();
				featureEstimation->setInputCloud(src2);
				featureEstimation->setInputNormals(src2);
				featureEstimation->compute(*feature2);

				corrEstimation->setInputSource(feature2);
				corrEstimation->setInputTarget(feature1);
				corrEstimation->determineCorrespondences(*allCorr, rate * 5);

				Eigen::Matrix4f mat;
				transEstimation->estimateRigidTransformation(*src2, *src1, *allCorr, mat);



				//icpFilter->setCorrespondenceEstimation(corrEstimation);
				//icpFilter->setInputSource(feature2);
				//icpFilter->setInputTarget(feature1);
				//icpFilter->align(*feature3);
				//auto mat = icpFilter->getLastIncrementalTransformation();

				pcl::transformPointCloud(*src2, *src3, mat);
				*src1 = (*src1) + (*src3);
			}
			downSampleFilter->setInputCloud(src1);
			downSampleFilter->filter(*dst);
		}
		m_curCameraPc.Set(dst);
	}
	void Switcher::ExecuteEventRegistarteRidarPointCloud(){
		TimePointType tp;
		PTNCPtr pc;
		m_lastLidarPc.Get(tp, pc);
		if (!pc || pc->size() == 0) return;


		m_curLidarPc.Set(pc);
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
			m_lastMachineState.GetRear().CurValue + diff;
		SetRearValue(CMD_VALUE_TYPE_TARGET, targetVal);
	}
	void Switcher::ChangeSteerValue(float diff) {
		float targetVal =
			diff == 0.0 ?
			0.0f :
			m_lastMachineState.GetSteer().CurValue + diff;
		SetSteerValue(CMD_VALUE_TYPE_TARGET, targetVal);
	}
	void Switcher::ChangeCameraPitchValue(float diff) {
		float targetVal =
			diff == 0.0 ?
			0.0f :
			m_lastMachineState.GetCameraPitch().CurValue + diff;
		SetCameraPitchValue(CMD_VALUE_TYPE_TARGET, targetVal);
	}
	void Switcher::ChangeCameraYawValue(float diff) {
		float targetVal =
			diff == 0.0 ?
			0.0f :
			m_lastMachineState.GetCameraYaw().CurValue + diff;
		SetCameraYawValue(CMD_VALUE_TYPE_TARGET, targetVal);
	}
}


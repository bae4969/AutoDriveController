#include "pch.h"
#include "Types.h"

namespace AutoDriveCode {
	using namespace std;
	using namespace cv;

	ImageData::ImageData() {
		W = 0;
		H = 0;
		Ch = 0;
		Step = 0;
		Data = NULL;
	}
	ImageData ImageData::Clone() {
		ImageData ret;

		ret.W = this->W;
		ret.H = this->H;
		ret.Ch = this->Ch;
		ret.Step = this->Step;

		size_t totMemSize = ret.W * ret.H * ret.Ch * ret.Step;
		if (totMemSize == 0)
			ret.Data = NULL;
		else {
			ret.Data = new char[totMemSize];
			memcpy(Data, this->Data, totMemSize);
		}

		return ret;
	}
	void ImageData::Update(Mat& mat) {
		if (mat.empty()) return;
		cvtColor(mat, mat, ColorConversionCodes::COLOR_RGB2RGBA);

		if (W == mat.cols &&
			H == mat.rows &&
			Ch == mat.elemSize() &&
			Step == mat.elemSize1())
		{
			size_t totMemSize = W * H * Ch * Step;
			memcpy(Data, mat.data, totMemSize);
		}
		else {
			W = mat.cols;
			H = mat.rows;
			Ch = mat.elemSize();
			Step = mat.elemSize1();

			size_t totMemSize = W * H * Ch * Step;
			if (Data)
				delete[] Data;
			Data = new char[totMemSize];
			memcpy(Data, mat.data, totMemSize);
		}
	}
	void ImageData::Release() {
		if (Data) {
			delete[] Data;
			Data = NULL;
		}
	}

	void StateType::Clone(StateType& state) {
		syncMutex.lock();
		state.Rear = Rear;
		state.Steer = Steer;
		state.CameraPitch = CameraPitch;
		state.CameraYaw = CameraYaw;
		state.SonicSensor = SonicSensor;
		state.FloorSensor = FloorSensor;
		state.FrameDateTime = FrameDateTime;
		state.OriginImage = OriginImage.clone();
		state.FilterImage = FilterImage.clone();
		syncMutex.unlock();
	}
	void StateType::UpdateMoveMotorState(MotorStateType<int>& rear, MotorStateType<float>& steer) {
		syncMutex.lock();
		Rear = rear;
		Steer = steer;
		syncMutex.unlock();
	}
	void StateType::UpdateCameraMotorState(MotorStateType<float>& cameraPitch, MotorStateType<float>& cameraYaw) {
		syncMutex.lock();
		CameraPitch = cameraPitch;
		CameraYaw = cameraYaw;
		syncMutex.unlock();
	}
	void StateType::UpdateSensorState(double& sonicSensor, std::vector<int>& floorSensor) {
		syncMutex.lock();
		SonicSensor = sonicSensor;
		FloorSensor = floorSensor;
		syncMutex.unlock();
	}
	void StateType::UpdateCameraImage(Mat& originImage, Mat& stateImage, Mat& filterImage) {
		auto now = chrono::steady_clock::now();
		syncMutex.lock();
		FrameDateTime.push(now);
		while ((now - FrameDateTime.front()) > chrono::seconds(1))
			FrameDateTime.pop();
		OriginImage = originImage;
		StateImage = stateImage;
		FilterImage = filterImage;
		syncMutex.unlock();
	}

	Mat StateType::GetOriginImage() {
		syncMutex.lock();
		Mat ret = OriginImage.clone();
		syncMutex.unlock();
		return ret;
	}
	Mat StateType::GetStateImage() {
		syncMutex.lock();
		Mat ret = StateImage.clone();
		syncMutex.unlock();
		return ret;
	}
	Mat StateType::GetFilterImage() {
		syncMutex.lock();
		Mat ret = FilterImage.clone();
		syncMutex.unlock();
		return ret;
	}

	MotorStateType<int> StateType::GetRear() {
		syncMutex.lock();
		MotorStateType<int> ret = Rear;
		syncMutex.unlock();
		return ret;
	}
	MotorStateType<float> StateType::GetSteer() {
		syncMutex.lock();
		MotorStateType<float> ret = Steer;
		syncMutex.unlock();
		return ret;
	}
	MotorStateType<float> StateType::GetCameraPitch() {
		syncMutex.lock();
		MotorStateType<float> ret = CameraPitch;
		syncMutex.unlock();
		return ret;
	}
	MotorStateType<float> StateType::GetCameraYaw() {
		syncMutex.lock();
		MotorStateType<float> ret = CameraYaw;
		syncMutex.unlock();
		return ret;
	}
}



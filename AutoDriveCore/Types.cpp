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

	void MachineStateType::Clone(MachineStateType& state) {
		syncMutex.lock();
		state.Rear = Rear;
		state.Steer = Steer;
		state.CameraPitch = CameraPitch;
		state.CameraYaw = CameraYaw;
		state.SonicSensor = SonicSensor;
		state.FloorSensor = FloorSensor;
		state.FrameDateTime = FrameDateTime;
		state.MachineTemperature = MachineTemperature;
		state.MachineStateBits = MachineStateBits;
		syncMutex.unlock();
	}
	void MachineStateType::UpdateMoveMotorState(MotorStateType<int>& rear, MotorStateType<float>& steer) {
		syncMutex.lock();
		Rear = rear;
		Steer = steer;
		syncMutex.unlock();
	}
	void MachineStateType::UpdateCameraMotorState(MotorStateType<float>& cameraPitch, MotorStateType<float>& cameraYaw) {
		syncMutex.lock();
		CameraPitch = cameraPitch;
		CameraYaw = cameraYaw;
		syncMutex.unlock();
	}
	void MachineStateType::UpdateSensorState(double& sonicSensor, std::vector<int>& floorSensor) {
		syncMutex.lock();
		SonicSensor = sonicSensor;
		FloorSensor = floorSensor;
		syncMutex.unlock();
	}
	void MachineStateType::UpdateLcdState(double& temp, int& state) {
		syncMutex.lock();
		MachineTemperature = temp;
		MachineStateBits = state;
		syncMutex.unlock();
	}
	void MachineStateType::UpdateFPS() {
		auto now = ClockType::now();
		syncMutex.lock();
		FrameDateTime.push(now);
		while ((now - FrameDateTime.front()) > chrono::seconds(1))
			FrameDateTime.pop();
		syncMutex.unlock();
	}

	MotorStateType<int> MachineStateType::GetRear() {
		syncMutex.lock();
		MotorStateType<int> ret = Rear;
		syncMutex.unlock();
		return ret;
	}
	MotorStateType<float> MachineStateType::GetSteer() {
		syncMutex.lock();
		MotorStateType<float> ret = Steer;
		syncMutex.unlock();
		return ret;
	}
	MotorStateType<float> MachineStateType::GetCameraPitch() {
		syncMutex.lock();
		MotorStateType<float> ret = CameraPitch;
		syncMutex.unlock();
		return ret;
	}
	MotorStateType<float> MachineStateType::GetCameraYaw() {
		syncMutex.lock();
		MotorStateType<float> ret = CameraYaw;
		syncMutex.unlock();
		return ret;
	}
	double MachineStateType::GetSonicValue() {
		syncMutex.lock();
		double ret = SonicSensor;
		syncMutex.unlock();
		return ret;
	}
	int MachineStateType::GetFloorValue(int idx) {
		syncMutex.lock();
		int ret = FloorSensor[idx];
		syncMutex.unlock();
		return ret;
	}
	int MachineStateType::GetFPS() {
		syncMutex.lock();
		int ret = FrameDateTime.size();
		syncMutex.unlock();
		return ret;
	}
	double MachineStateType::GetTemperature() {
		syncMutex.lock();
		double ret = MachineTemperature;
		syncMutex.unlock();
		return ret;
	}
	int MachineStateType::GetStateBits() {
		syncMutex.lock();
		int ret = MachineStateBits;
		syncMutex.unlock();
		return ret;
	}


	template void DeltaType<Mat>::Set(DeltaType<Mat>& delta);
	template void DeltaType<PTLNCPtr>::Set(DeltaType<PTLNCPtr>& delta);
	template void DeltaType<PTNCPtr>::Set(DeltaType<PTNCPtr>& delta);
	template<typename TYPE> void DeltaType<TYPE>::Set(DeltaType<TYPE>& delta) {
		syncMutex.lock();
		delta.syncMutex.lock();
		Time = delta.Time;
		Data = delta.Data;
		delta.syncMutex.unlock();
		syncMutex.unlock();
	}
	template void DeltaType<Mat>::Set(Mat delta);
	template void DeltaType<PTLNCPtr>::Set(PTLNCPtr delta);
	template void DeltaType<PTNCPtr>::Set(PTNCPtr delta);
	template<typename TYPE> void DeltaType<TYPE>::Set(TYPE data) {
		syncMutex.lock();
		Time = ClockType::now();
		Data = data;
		syncMutex.unlock();
	}
	template void DeltaType<Mat>::Get(TimePointType& time, Mat& delta);
	template void DeltaType<PTLNCPtr>::Get(TimePointType& time, PTLNCPtr& delta);
	template void DeltaType<PTNCPtr>::Get(TimePointType& time, PTNCPtr& delta);
	template<typename TYPE> void DeltaType<TYPE>::Get(TimePointType& time, TYPE& data) {
		syncMutex.lock();
		time = Time;
		data = Data;
		syncMutex.unlock();
	}
}



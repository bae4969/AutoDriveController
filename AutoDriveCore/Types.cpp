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
	void MachineStateType::UpdateFPS() {
		auto now = chrono::steady_clock::now();
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

	void ImageStateType::UpdateOriginImage(Mat& originImage) {
		syncMutex.lock();
		OriginImage = originImage;
		syncMutex.unlock();
	}
	void ImageStateType::UpdateStateImage(Mat& stateImage) {
		syncMutex.lock();
		StateImage.release();
		StateImage = stateImage;
		syncMutex.unlock();
	}
	void ImageStateType::UpdateFilterImage(Mat& filterImage) {
		syncMutex.lock();
		FilterImage.release();
		FilterImage = filterImage;
		syncMutex.unlock();
	}
	void ImageStateType::UpdatePointCloud(PTLCPtr& pointCloud) {
		syncMutex.lock();
		PointCloud = pointCloud;
		syncMutex.unlock();
	}

	Mat ImageStateType::GetOriginImage() {
		syncMutex.lock();
		Mat ret = OriginImage.clone();
		syncMutex.unlock();
		return ret;
	}
	Mat ImageStateType::GetStateImage() {
		syncMutex.lock();
		Mat ret = StateImage.clone();
		syncMutex.unlock();
		return ret;
	}
	Mat ImageStateType::GetFilterImage() {
		syncMutex.lock();
		Mat ret = FilterImage.clone();
		syncMutex.unlock();
		return ret;
	}
	PTLCPtr ImageStateType::GetPointCloud() {
		syncMutex.lock();
		PTLCPtr ret(PointCloud);
		syncMutex.unlock();
		return ret;
	}

	void CameraCaliDataType::Init() {
		Margin = 4;
		RollDegree = -2.1;

		PointCloudSize.width = 640 - Margin;
		PointCloudSize.height = 480 - Margin;


		float imgXDistanceRate = 1.0f;
		float imgYDistanceRate = 0.7f;

		float distance = 100.f;
		float x_scale = imgXDistanceRate / PointCloudSize.width * distance;
		float z_scale = imgYDistanceRate / PointCloudSize.height * distance;
		float x_offset = -PointCloudSize.width * 0.5f;
		float z_offset = -PointCloudSize.height * 0.5f;

		PTCPtr t_offset(new PTCType());
		t_offset->resize(PointCloudSize.area());
		for (int y = 0; y < PointCloudSize.height; y++) {
			for (int x = 0; x < PointCloudSize.width; x++) {
				size_t idx = y * PointCloudSize.width + x;
				PTType& pt = t_offset->at(idx);
				pt.x = (x_offset + x) * x_scale;
				pt.y = distance;
				pt.z = -(z_offset + y) * z_scale;

				float scalrValue = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
				float multiValue = distance / scalrValue;
				pt.x *= multiValue;
				pt.y *= multiValue;
				pt.z *= multiValue;
			}
		}
		PointOffsets = t_offset;
	}
}



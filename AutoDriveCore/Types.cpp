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

	void CaliDataType::Init() {
		Margin = IMAGE_EDGE_CUT_MARGIN;
		RollDegree = -2.1;

		PointCloudSize.width = (CAMERA_IMAGE_WIDTH - (2 * Margin)) * IMAGE_TO_POINT_SIZE_RATE;
		PointCloudSize.height = (CAMERA_IMAGE_HEIGHT - (2 * Margin)) * IMAGE_TO_POINT_SIZE_RATE;

		float x_scale = CAMERA_IMAGE_X_DISTANCE_RATE * IMAGE_POINT_DEFAULT_DISTANCE / PointCloudSize.width;
		float z_scale = CAMERA_IMAGE_Y_DISTANCE_RATE * IMAGE_POINT_DEFAULT_DISTANCE / PointCloudSize.height;
		float x_offset = -PointCloudSize.width * 0.5f;
		float z_offset = -PointCloudSize.height * 0.5f;

		PTCPtr t_offset(new PTCType());
		t_offset->resize(PointCloudSize.area());
		for (int y = 0; y < PointCloudSize.height; y++) {
			for (int x = 0; x < PointCloudSize.width; x++) {
				size_t idx = y * PointCloudSize.width + x;
				PTType& pt = t_offset->at(idx);
				pt.x = (x_offset + x) * x_scale;
				pt.y = IMAGE_POINT_DEFAULT_DISTANCE;
				pt.z = -(z_offset + y) * z_scale;

				float scalrValue = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
				float multiValue = IMAGE_POINT_DEFAULT_DISTANCE / scalrValue;
				pt.x *= multiValue;
				pt.y *= multiValue;
				pt.z *= multiValue;
			}
		}
		PointOffsets = t_offset;

		Eigen::AngleAxisd roll(DEGREE_TO_RADIAN(RollDegree), -Eigen::Vector3d::UnitY());
		Eigen::Matrix3d rotationMatrix = roll.matrix();
		Eigen::Matrix4f transMat = Eigen::Matrix4f::Identity();
		for (int i = 0; i < 3; i++)for (int j = 0; j < 3; j++)
			transMat.col(i)[j] = rotationMatrix.col(i)[j];

		PointOffsets = make_shared<PTCType>();
		pcl::transformPointCloud(*t_offset, *PointOffsets, transMat);
	}


	void DeltaImageType::Set(DeltaImageType& deltaImage) {
		syncMutex.lock();
		deltaImage.syncMutex.lock();
		Time = deltaImage.Time;
		Image = deltaImage.Image;
		PitchDegree = deltaImage.PitchDegree;
		YawDegree = deltaImage.YawDegree;
		deltaImage.syncMutex.unlock();
		syncMutex.unlock();
	}
	void DeltaImageType::Set(Mat img, float pitch, float yaw) {
		syncMutex.lock();
		Time = ClockType::now();
		Image = img;
		PitchDegree = pitch;
		YawDegree = yaw;
		syncMutex.unlock();
	}
	void DeltaImageType::Get(ClockType::time_point& time, Mat& img, float& pitch, float& yaw) {
		syncMutex.lock();
		time = Time;
		img = Image;
		pitch = PitchDegree;
		yaw = YawDegree;
		syncMutex.unlock();
	}

	void DeltaLidarPoint::Set(PTCPtr pc) {
		syncMutex.lock();
		Time = ClockType::now();
		LidarPoints = pc;
		syncMutex.unlock();
	}
	void DeltaLidarPoint::Get(ClockType::time_point& time, PTCPtr& pc) {
		syncMutex.lock();
		time = Time;
		pc = LidarPoints;
		syncMutex.unlock();
	}
}



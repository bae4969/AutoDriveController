using NetMQ;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoDriveControlor.Classes
{
	public struct MotorState<TYPE>
	{
		public TYPE CurValue;
		public TYPE TarValue;
		public TYPE Speed;
	}
	public struct StateValues
	{
		public object LockObj;
		public MotorState<int> Rear;
		public MotorState<float> Steer;
		public MotorState<float> CameraPitch;
		public MotorState<float> CameraYaw;

		public double SonicSensor;
		public int[] FloorSensor;

		public StateValues()
		{
			LockObj = new();

			Rear = new();
			Steer = new();
			CameraPitch = new();
			CameraYaw = new();

			SonicSensor = 0.0;
			FloorSensor = new int[3];
		}
		public StateValues Clone()
		{
			StateValues ret = new()
			{
				LockObj = new(),
				FloorSensor = new int[3]
			};
			lock (LockObj)
			{
				ret.Rear = Rear;
				ret.Steer = Steer;
				ret.CameraPitch = CameraPitch;
				ret.CameraYaw = CameraYaw;
				ret.SonicSensor = SonicSensor;
				ret.FloorSensor[0] = FloorSensor[0];
				ret.FloorSensor[1] = FloorSensor[1];
				ret.FloorSensor[2] = FloorSensor[2];
			}
			return ret;
		}
		public bool UpdateRearMotorMessage(NetMQMessage msg)
		{
			try
			{
				int rearCur = BitConverter.ToInt32(msg.Pop().Buffer);
				int rearTar = BitConverter.ToInt32(msg.Pop().Buffer);
				int rearSpd = BitConverter.ToInt32(msg.Pop().Buffer);
				float steerCur = BitConverter.ToSingle(msg.Pop().Buffer);
				float steerTar = BitConverter.ToSingle(msg.Pop().Buffer);
				float steerSpd = BitConverter.ToSingle(msg.Pop().Buffer);

				lock (LockObj)
				{
					Rear.CurValue = rearCur;
					Rear.TarValue = rearTar;
					Rear.Speed = rearSpd;
					Steer.CurValue = steerCur;
					Steer.TarValue = steerTar;
					Steer.Speed = steerSpd;
				}

				return true;
			}
			catch { return false; }
		}
		public bool UpdateCameraMotorMessage(NetMQMessage msg)
		{
			try
			{
				float pitchCur = BitConverter.ToSingle(msg.Pop().Buffer);
				float pitchTar = BitConverter.ToSingle(msg.Pop().Buffer);
				float pitchSpd = BitConverter.ToSingle(msg.Pop().Buffer);
				float yawCur = BitConverter.ToSingle(msg.Pop().Buffer);
				float yawTar = BitConverter.ToSingle(msg.Pop().Buffer);
				float yawSpd = BitConverter.ToSingle(msg.Pop().Buffer);

				lock (LockObj)
				{
					CameraPitch.CurValue = pitchCur;
					CameraPitch.TarValue = pitchTar;
					CameraPitch.Speed = pitchSpd;
					CameraYaw.CurValue = yawCur;
					CameraYaw.TarValue = yawTar;
					CameraYaw.Speed = yawSpd;
				}

				return true;
			}
			catch { return false; }
		}
		public bool UpdateSensorMessage(NetMQMessage msg)
		{
			try
			{
				double sonic = BitConverter.ToDouble(msg.Pop().Buffer);
				int floorLeft = BitConverter.ToInt32(msg.Pop().Buffer);
				int floorCenter = BitConverter.ToInt32(msg.Pop().Buffer);
				int floorRight = BitConverter.ToInt32(msg.Pop().Buffer);

				lock (LockObj)
				{
					SonicSensor = sonic;
					FloorSensor[0] = floorLeft;
					FloorSensor[1] = floorCenter;
					FloorSensor[2] = floorRight;
				}

				return true;
			}
			catch { return false; }
		}
	}
}

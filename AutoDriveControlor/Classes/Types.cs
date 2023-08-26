using NetMQ;
using System;
using System.Collections.Generic;
using System.Drawing.Imaging;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace AutoDriveControlor.Classes
{
	public struct MotorState<TYPE>
	{
		public TYPE CurValue;
		public TYPE TarValue;
		public TYPE Speed;

		public MotorState<TYPE> Clone()
		{
			return new()
			{
				CurValue = this.CurValue,
				TarValue = this.TarValue,
				Speed = this.Speed,
			};
		}
	}
	public class StateValues
	{
		private object LockMoveMotorObj;
		private object LockCameraMotorObj;
		private object LockSensorObj;
		private object LockCameraObj;


		private MotorState<int> Rear;
		private MotorState<float> Steer;
		private MotorState<float> CameraPitch;
		private MotorState<float> CameraYaw;

		private double SonicSensor;
		private List<int> FloorSensor;

		private Queue<DateTime> LastFrameDt;
		private int FrameRate;
		private Bitmap ImageFrame;

		public StateValues()
		{
			LockMoveMotorObj = new();
			LockCameraMotorObj = new();
			LockSensorObj = new();
			LockCameraObj = new();

			Rear = new();
			Steer = new();
			CameraPitch = new();
			CameraYaw = new();

			SonicSensor = 0.0;
			FloorSensor = new()
			{
				0,
				0,
				0
			};

			LastFrameDt = new();
			FrameRate = 0;
			ImageFrame = new Bitmap(100, 100, PixelFormat.Format24bppRgb);
		}
		public StateValues Clone()
		{
			StateValues ret = new()
			{
				LockMoveMotorObj = new(),
				LockCameraMotorObj = new(),
				LockSensorObj = new(),
				LockCameraObj = new(),
			};

			lock (LockMoveMotorObj)
			{
				ret.Rear = Rear;
				ret.Steer = Steer;
			}

			lock (LockCameraMotorObj)
			{
				ret.CameraPitch = CameraPitch;
				ret.CameraYaw = CameraYaw;
			}

			lock (LockSensorObj)
			{
				ret.SonicSensor = SonicSensor;
				ret.FloorSensor = FloorSensor;
			}

			lock (LockCameraObj)
			{
				ret.LastFrameDt = LastFrameDt;
				ret.FrameRate = FrameRate;
				ret.ImageFrame = (Bitmap)ImageFrame.Clone();
			}

			return ret;
		}

		public bool UpdateMoveMotorMessage(NetMQMessage msg)
		{
			try
			{
				int rearCur = BitConverter.ToInt32(msg.Pop().Buffer);
				int rearTar = BitConverter.ToInt32(msg.Pop().Buffer);
				int rearSpd = BitConverter.ToInt32(msg.Pop().Buffer);
				float steerCur = BitConverter.ToSingle(msg.Pop().Buffer);
				float steerTar = BitConverter.ToSingle(msg.Pop().Buffer);
				float steerSpd = BitConverter.ToSingle(msg.Pop().Buffer);

				lock (LockMoveMotorObj)
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

				lock (LockCameraMotorObj)
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

				lock (LockSensorObj)
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
		public bool UpdateCameraSensorMessage(NetMQMessage msg)
		{
			try
			{
				//var ms = new MemoryStream(msg[1].Buffer);
				//Bitmap bmp = Bitmap.FromStream(ms);

				int w = BitConverter.ToInt32(msg.Pop().Buffer);
				int h = BitConverter.ToInt32(msg.Pop().Buffer);
				int channel = BitConverter.ToInt32(msg.Pop().Buffer);
				byte[] data = msg.Pop().Buffer;
				if (channel != 3) throw new Exception();

				Rectangle imgRect = new(0, 0, w, h);
				Bitmap bmp = new(w, h, PixelFormat.Format24bppRgb);
				BitmapData bmpData = bmp.LockBits(
					imgRect,
					ImageLockMode.WriteOnly,
					bmp.PixelFormat
				);

				IntPtr pNative = bmpData.Scan0;
				Marshal.Copy(data, 0, pNative, data.Length);
				bmp.UnlockBits(bmpData);

				lock (LockCameraObj)
				{
					Image beforeImage = ImageFrame;
					ImageFrame = bmp;
					beforeImage?.Dispose();

					LastFrameDt.Enqueue(DateTime.Now);
					if (LastFrameDt.Count > 11)
						LastFrameDt.Dequeue();

					TimeSpan totalSpan = TimeSpan.Zero;
					for (int i = 0; i + 1 < LastFrameDt.Count; i++)
						totalSpan += LastFrameDt.ElementAt(i + 1) - LastFrameDt.ElementAt(i);

					FrameRate =
						totalSpan.Milliseconds != 0 ?
						10000 / totalSpan.Milliseconds :
						0;
				}
				return true;
			}
			catch { return false; }
		}

		public MotorState<int> GetRear()
		{
			lock (LockMoveMotorObj)
			{
				return Rear.Clone();
			}
		}
		public MotorState<float> GetSteer()
		{
			lock (LockMoveMotorObj)
			{
				return Steer.Clone();
			}
		}
		public MotorState<float> GetCameraPitch()
		{
			lock (LockCameraMotorObj)
			{
				return CameraPitch.Clone();
			}
		}
		public MotorState<float> GetCameraYaw()
		{
			lock (LockCameraMotorObj)
			{
				return CameraYaw.Clone();
			}
		}
		public double GetSonicSensor()
		{
			lock (LockSensorObj)
			{
				return SonicSensor;
			}
		}
		public List<int> GetFloorSensor()
		{
			lock (LockSensorObj)
			{
				return FloorSensor;
			}
		}
		public int GetFrameRate()
		{
			lock (LockCameraObj)
			{
				return FrameRate;
			}
		}
		public Bitmap GetBitmap()
		{
			lock (LockCameraObj)
			{
				return (Bitmap)ImageFrame.Clone();
			}
		}
	}
}

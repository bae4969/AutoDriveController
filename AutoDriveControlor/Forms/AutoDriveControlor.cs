using NetMQ;
using NetMQ.Sockets;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Text;
using System.Windows.Forms;
using AutoDriveControlor.Classes;

namespace AutoDriveControlor.Forms
{
	public partial class AutoDriveControlor : Form
	{
		Task NetMQTask;
		NetMQPoller MainPoller = new();
		NetMQPoller PbPoller = new();
		NetMQQueue<Bitmap> FrameQueue = new();
		NetMQQueue<NetMQMessage> CommandQueue = new();
		NetMQTimer ConnectionTimer = new(TimeSpan.FromSeconds(1));
		PublisherSocket CommandPub = new();
		SubscriberSocket StateSub = new();
		SubscriberSocket CameraSub = new();
		StateValues CurrentState = new();

		void NetMQThreadFunc()
		{
			FrameQueue.ReceiveReady += FrameQueueFunc;
			CommandQueue.ReceiveReady += CommandQueueFunc;
			ConnectionTimer.Elapsed += ConnectionTimerFunc;
			StateSub.ReceiveReady += StateSubFunc;
			CameraSub.ReceiveReady += CameraSubFunc;

			StateSub.Subscribe("STATE_MOVE_MOTOR");
			StateSub.Subscribe("STATE_CAMERA_MOTOR");
			StateSub.Subscribe("STATE_SENSOR");
			CameraSub.Subscribe("STATE_CAMERA_SENSOR");

			CommandPub.Connect(CONNECTION_STRINGS.PUB);
			StateSub.Connect(CONNECTION_STRINGS.SUB);
			CameraSub.Connect(CONNECTION_STRINGS.SUB);

			PbPoller.Add(FrameQueue);
			MainPoller.Add(CommandQueue);
			MainPoller.Add(ConnectionTimer);
			MainPoller.Add(CommandPub);
			MainPoller.Add(StateSub);
			MainPoller.Add(CameraSub);
			PbPoller.RunAsync();
			MainPoller.Run();

			CommandPub.Disconnect(CONNECTION_STRINGS.PUB);
			StateSub.Disconnect(CONNECTION_STRINGS.SUB);
			CameraSub.Disconnect(CONNECTION_STRINGS.SUB);
		}

		void FrameQueueFunc(object? s, NetMQQueueEventArgs<Bitmap> e)
		{
			try
			{
				Bitmap frame = e.Queue.Dequeue();
				while (e.Queue.Count > 0)
					frame = e.Queue.Dequeue();

				PB_CameraView.Invoke(new Action(() => { PB_CameraView.Image = frame; }));
			}
			catch { }
		}
		void CommandQueueFunc(object? s, NetMQQueueEventArgs<NetMQMessage> e)
		{
			try
			{
				NetMQMessage msg = e.Queue.Dequeue();
				while (e.Queue.Count > 0)
					msg = e.Queue.Dequeue();

				CommandPub.SendMultipartMessage(msg);
			}
			catch { }
		}
		void ConnectionTimerFunc(object? s, NetMQTimerEventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_PICAR", Encoding.ASCII);
			msg.Append("UPDATE_CONNECTION", Encoding.ASCII);
			CommandQueue.Enqueue(msg);
		}
		void StateSubFunc(object? s, NetMQSocketEventArgs e)
		{
			try
			{
				NetMQMessage msg = e.Socket.ReceiveMultipartMessage();
				string topic = msg.Pop().ConvertToString(Encoding.ASCII);
				switch (topic)
				{
					case "STATE_MOVE_MOTOR":
						CurrentState.UpdateRearMotorMessage(msg);
						break;
					case "STATE_CAMERA_MOTOR":
						CurrentState.UpdateCameraMotorMessage(msg);
						break;
					case "STATE_SENSOR":
						CurrentState.UpdateSensorMessage(msg);
						break;
				}
			}
			catch { }
		}
		void CameraSubFunc(object? s, NetMQSocketEventArgs e)
		{
			try
			{
				var msg = e.Socket.ReceiveMultipartMessage();

				string topic = msg[0].ConvertToString(Encoding.ASCII);
				int w = BitConverter.ToInt32(msg[1].Buffer);
				int h = BitConverter.ToInt32(msg[2].Buffer);
				int channel = BitConverter.ToInt32(msg[3].Buffer);
				byte[] data = msg[4].Buffer;

				var bmp = new Bitmap(w, h, PixelFormat.Format24bppRgb);
				BitmapData bmpData = bmp.LockBits(new Rectangle(0, 0,
															bmp.Width,
															bmp.Height),
											  ImageLockMode.WriteOnly,
											  bmp.PixelFormat);

				IntPtr pNative = bmpData.Scan0;
				Marshal.Copy(data, 0, pNative, data.Length);
				bmp.UnlockBits(bmpData);

				FrameQueue.Enqueue(bmp);
			}
			catch { }
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		public AutoDriveControlor()
		{
			InitializeComponent();
		}
		private void AutoDriveControlor_Load(object sender, EventArgs e)
		{
			NetMQTask = Task.Run(() => { NetMQThreadFunc(); });
		}
		private void AutoDriveControlor_FormClosing(object sender, FormClosingEventArgs e)
		{
			MainPoller.Stop();
			NetMQTask.Wait();
		}

		private void BTN_TurnOff_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_PICAR", Encoding.ASCII);
			msg.Append("TURN_OFF", Encoding.ASCII);
			CommandQueue.Enqueue(msg);
		}
		private void BTN_Stop_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("STOP", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0));
			CommandQueue.Enqueue(msg);
		}

		private void BTN_ResetRear_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_RearFront_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.Rear.CurValue + 100));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_RearBack_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.Rear.CurValue - 100));
			CommandQueue.Enqueue(msg);
		}

		private void BTN_ResetSteer_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("STEER_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0.0f));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_SteerRight_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("STEER_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.Steer.CurValue + 5.0f));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_SteerLeft_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("STEER_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.Steer.CurValue - 5.0f));
			CommandQueue.Enqueue(msg);
		}

		private void BTN_ResetPitch_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("PITCH_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0.0f));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_PitchUp_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("PITCH_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.CameraPitch.CurValue + 10.0f));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_PitchDown_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("PITCH_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.CameraPitch.CurValue - 10.0f));
			CommandQueue.Enqueue(msg);
		}

		private void BTN_ResetYaw_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("YAW_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0.0f));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_YawRight_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("YAW_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.CameraYaw.CurValue + 10.0f));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_YawLeft_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("YAW_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.CameraYaw.CurValue - 10.0f));
			CommandQueue.Enqueue(msg);
		}

	}
}
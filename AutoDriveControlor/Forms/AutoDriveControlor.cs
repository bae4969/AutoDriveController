using NetMQ;
using NetMQ.Sockets;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Text;
using System.Windows.Forms;
using AutoDriveControlor.Classes;
using System.Drawing;
using System.IO;
using System.Diagnostics;

namespace AutoDriveControlor.Forms
{
	public partial class AutoDriveControlor : Form
	{
		Task NetMQTask;
		NetMQPoller MainPoller = new();
		NetMQTimer ConnectionTimer = new(TimeSpan.FromSeconds(1));
		NetMQQueue<NetMQMessage> CommandQueue = new();
		PublisherSocket CommandPub = new();
		SubscriberSocket StateSub = new();

		StateValues CurrentState = new();

		private void NetMQThreadFunc()
		{
			ConnectionTimer.Elapsed += ConnectionTimerFunc;
			CommandQueue.ReceiveReady += CommandQueueFunc;
			StateSub.ReceiveReady += StateSubFunc;

			StateSub.Subscribe("STATE_MOVE_MOTOR");
			StateSub.Subscribe("STATE_CAMERA_MOTOR");
			StateSub.Subscribe("STATE_SENSOR");
			StateSub.Subscribe("STATE_CAMERA_SENSOR");

			CommandPub.Connect(CONNECTION_STRINGS.PUB);
			StateSub.Connect(CONNECTION_STRINGS.SUB);

			MainPoller = new()
			{
				ConnectionTimer,
				CommandQueue,
				CommandPub,
				StateSub,
			};
			MainPoller.Run();

			CommandPub.Disconnect(CONNECTION_STRINGS.PUB);
			StateSub.Disconnect(CONNECTION_STRINGS.SUB);
		}

		private void CommandQueueFunc(object? s, NetMQQueueEventArgs<NetMQMessage> e)
		{
			try
			{
				NetMQMessage msg = e.Queue.Dequeue();
				while (e.Queue.Count > 0)
					msg = e.Queue.Dequeue();

				CommandPub.SendMultipartMessage(msg);
			}
			finally
			{
				GC.Collect();
			}
		}
		private void ConnectionTimerFunc(object? s, NetMQTimerEventArgs e)
		{
			try
			{
				NetMQMessage msg = new NetMQMessage();
				msg.Append("COMMAND_PICAR", Encoding.ASCII);
				msg.Append("UPDATE_CONNECTION", Encoding.ASCII);
				CommandQueue.Enqueue(msg);
			}
			finally
			{
				GC.Collect();
			}
		}
		private void StateSubFunc(object? s, NetMQSocketEventArgs e)
		{
			try
			{
				NetMQMessage msg = e.Socket.ReceiveMultipartMessage();
				string topic = msg.Pop().ConvertToString(Encoding.ASCII);
				switch (topic)
				{
					case "STATE_MOVE_MOTOR":
						CurrentState.UpdateMoveMotorMessage(msg);
						break;
					case "STATE_CAMERA_MOTOR":
						CurrentState.UpdateCameraMotorMessage(msg);
						break;
					case "STATE_SENSOR":
						CurrentState.UpdateSensorMessage(msg);
						break;
					case "STATE_CAMERA_SENSOR":
						CurrentState.UpdateCameraSensorMessage(msg);
						break;
				}
			}
			finally
			{
				GC.Collect();
			}
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		Task CameraViewTask;
		bool IsStop = false;

		private void UpdateCameraViewThreaFunc()
		{
			while (!IsStop)
			{
				TB_FPS.Invoke(delegate ()
				{
					TB_FPS.Text = CurrentState.GetFrameRate().ToString();
				});
				PB_CameraView.Invoke(delegate ()
				{
					Image beforeImage = PB_CameraView.Image;
					PB_CameraView.Image = CurrentState.GetBitmap();
					beforeImage?.Dispose();
				});

				Thread.Sleep(TimeSpan.FromMilliseconds(10));
			}
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		public AutoDriveControlor()
		{
			InitializeComponent();
		}
		private void AutoDriveControlor_Load(object sender, EventArgs e)
		{
			NetMQTask = Task.Run(() => { NetMQThreadFunc(); });
			CameraViewTask = Task.Run(() => { UpdateCameraViewThreaFunc(); });
		}
		private async void AutoDriveControlor_FormClosing(object sender, FormClosingEventArgs e)
		{
			IsStop = true;
			MainPoller.Stop();
			await NetMQTask;
			await CameraViewTask;
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
			msg.Append(BitConverter.GetBytes(CurrentState.GetRear().CurValue + 100));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_RearBack_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetRear().CurValue - 100));
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
			msg.Append(BitConverter.GetBytes(CurrentState.GetSteer().CurValue + 5.0f));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_SteerLeft_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("STEER_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetSteer().CurValue - 5.0f));
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
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraPitch().CurValue + 10.0f));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_PitchDown_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("PITCH_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraPitch().CurValue - 10.0f));
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
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraYaw().CurValue + 10.0f));
			CommandQueue.Enqueue(msg);
		}
		private void BTN_YawLeft_Click(object sender, EventArgs e)
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("YAW_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraYaw().CurValue - 10.0f));
			CommandQueue.Enqueue(msg);
		}
	}
}
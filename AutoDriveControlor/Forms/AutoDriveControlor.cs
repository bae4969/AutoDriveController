using NetMQ;
using NetMQ.Sockets;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Text;
using System.Windows.Forms;
using AutoDriveControlor.Classes;
using AutoDriveControlor.Imports;
using System.Drawing;
using System.IO;
using System.Diagnostics;
using Microsoft.VisualBasic.Devices;
using static System.Net.WebRequestMethods;
using System;

namespace AutoDriveControlor.Forms
{
	public partial class AutoDriveControlor : Form
	{
		private Task? NetMQTask = null;
		private Proxy? NetMQProxy = null;
		private NetMQTimer ConnectionTimer = new(TimeSpan.FromSeconds(1));
		private NetMQQueue<KeyValuePair<bool, NetMQMessage>> CommandQueue = new();
		private XPublisherSocket xPub = new();
		private XSubscriberSocket xSub = new();
		private PublisherSocket CommandPub = new();
		private SubscriberSocket StateSub = new();

		private StateValues CurrentState = new();

		private void NetMQThreadFunc()
		{
			if (!Directory.Exists("./temp"))
				Directory.CreateDirectory("./temp");

			xPub.Options.SendHighWatermark = 20;
			xSub.Options.ReceiveHighWatermark = 20;
			CommandPub.Options.SendHighWatermark = 20;
			StateSub.Options.ReceiveHighWatermark = 20;

			xSub.Connect(CONNECTION_STRINGS.EXTERN_PUB);
			xPub.Bind(CONNECTION_STRINGS.LOCAL_PUB);
			CommandPub.Connect(CONNECTION_STRINGS.EXTERN_SUB);
			StateSub.Connect(CONNECTION_STRINGS.LOCAL_PUB);

			ConnectionTimer.Elapsed += ConnectionTimerFunc;
			CommandQueue.ReceiveReady += CommandQueueFunc;
			StateSub.ReceiveReady += StateSubFunc;

			StateSub.Subscribe("STATE_MOVE_MOTOR");
			StateSub.Subscribe("STATE_CAMERA_MOTOR");
			StateSub.Subscribe("STATE_SENSOR");

			NetMQProxy = new(xSub, xPub);
			NetMQPoller LocalPoller = new()
				{
					ConnectionTimer,
					CommandQueue,
					CommandPub,
					StateSub,
				};

			LocalPoller.RunAsync();
			NetMQProxy.Start();
			LocalPoller.Stop();

			CommandPub.Disconnect(CONNECTION_STRINGS.EXTERN_SUB);
			StateSub.Disconnect(CONNECTION_STRINGS.LOCAL_PUB);
			xSub.Disconnect(CONNECTION_STRINGS.EXTERN_PUB);
			xPub.Unbind(CONNECTION_STRINGS.LOCAL_PUB);
		}

		private void CommandQueueFunc(object? s, NetMQQueueEventArgs<KeyValuePair<bool, NetMQMessage>> e)
		{
			try
			{
				KeyValuePair<bool, NetMQMessage> val = e.Queue.Dequeue();
				while (!val.Key && e.Queue.Count > 5)
					val = e.Queue.Dequeue();

				CommandPub.SendMultipartMessage(val.Value);
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

				KeyValuePair<bool, NetMQMessage> val = new(true, msg);
				CommandQueue.Enqueue(val);
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
				}
			}
			finally
			{
				GC.Collect();
			}
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		private bool IsStopToUpdateCamera = false;
		private Task? CameraViewTask = null;
		private PictureBox? ZoomInPb = null;
		private Bitmap? OriginImage = new(100, 100);
		private Bitmap? FilterImage = new(100, 100);

		private void UpdateCameraViewThreadFunc()
		{
			while (!IsStopToUpdateCamera)
			{
				DateTime start = DateTime.Now;

				Core.ImageData? originData = Core.GetOriginImage();
				Core.ImageData? filterData = Core.GetFilterImage();
				OriginImage = originData?.ToBitmap();
				FilterImage = filterData?.ToBitmap();
				PB_ViewTL.Invalidate();
				PB_ViewTR.Invalidate();

				if ((DateTime.Now - start).Milliseconds < 33)
					Thread.Sleep(TimeSpan.FromMilliseconds(33) - (DateTime.Now - start));
			}
		}

		private void PB_DoubleClickEvent(object sender, EventArgs e)
		{
			PictureBox pb = (PictureBox)sender;
			if (ZoomInPb == null)
				ZoomInPb = pb;
			else if (ZoomInPb == pb)
				ZoomInPb = null;
			else
				ZoomInPb = pb;


			if (ZoomInPb == PB_ViewTL || ZoomInPb == PB_ViewTR)
			{
				TLP_ViewLayoutTable.RowStyles[0].Height = TLP_ViewLayoutTable.Height;
				TLP_ViewLayoutTable.RowStyles[1].Height = 0;
			}
			else if (ZoomInPb == PB_ViewBL || ZoomInPb == PB_ViewBR)
			{
				TLP_ViewLayoutTable.RowStyles[0].Height = 0;
				TLP_ViewLayoutTable.RowStyles[1].Height = TLP_ViewLayoutTable.Height;
			}
			else
			{
				TLP_ViewLayoutTable.RowStyles[0].Height = TLP_ViewLayoutTable.RowStyles[1].Height = TLP_ViewLayoutTable.Height * 0.5f;
			}

			if (ZoomInPb == PB_ViewTL || ZoomInPb == PB_ViewBL)
			{
				TLP_ViewLayoutTable.ColumnStyles[0].Width = TLP_ViewLayoutTable.Width;
				TLP_ViewLayoutTable.ColumnStyles[1].Width = 0;
			}
			else if (ZoomInPb == PB_ViewTR || ZoomInPb == PB_ViewBR)
			{
				TLP_ViewLayoutTable.ColumnStyles[0].Width = 0;
				TLP_ViewLayoutTable.ColumnStyles[1].Width = TLP_ViewLayoutTable.Width;
			}
			else
			{
				TLP_ViewLayoutTable.ColumnStyles[0].Width = TLP_ViewLayoutTable.ColumnStyles[1].Width = TLP_ViewLayoutTable.Width * 0.5f;
			}
		}
		private void PB_ViewTL_Paint(object sender, PaintEventArgs e)
		{
			if (OriginImage == null) return;

			PictureBox targetPB = (PictureBox)sender;
			RectangleF zommImgRect = PB_CAL.CalZoomImagePictureBoxRectangle(targetPB.ClientRectangle, OriginImage.Size);
			e.Graphics.DrawImage(OriginImage, zommImgRect);
		}
		private void PB_ViewTR_Paint(object sender, PaintEventArgs e)
		{
			if (FilterImage == null) return;

			PictureBox targetPB = (PictureBox)sender;
			RectangleF zommImgRect = PB_CAL.CalZoomImagePictureBoxRectangle(targetPB.ClientRectangle, FilterImage.Size);
			e.Graphics.DrawImage(FilterImage, zommImgRect);
		}
		private void PB_ViewBL_Paint(object sender, PaintEventArgs e)
		{

		}
		private void PB_ViewBR_Paint(object sender, PaintEventArgs e)
		{

		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		private bool IsStopToUpdateCommand = false;
		private Task? CommandTask = null;
		private Action? SpecialFunc = null;
		private Action? RearFunc = null;
		private Action? SteerFunc = null;
		private Action? PitchFunc = null;
		private Action? YawFunc = null;
		private TimeSpan DELTA_DURATION = TimeSpan.FromMilliseconds(100);

		private void UpdateCommandThreadFunc()
		{
			while (!IsStopToUpdateCommand)
			{
				if (SpecialFunc != null)
					SpecialFunc();
				if (RearFunc != null)
					RearFunc();
				if (SteerFunc != null)
					SteerFunc();
				if (PitchFunc != null)
					PitchFunc();
				if (YawFunc != null)
					YawFunc();
				Thread.Sleep(DELTA_DURATION);
			}
		}

		private void CommonKeyDown(object sender, KeyEventArgs e)
		{
			switch (e.KeyCode)
			{
				case Keys.Q:
					SpecialFunc = CommandTurnOff;
					break;
				case Keys.E:
					RearFunc = CommandStopNow;
					break;

				case Keys.X:
					RearFunc = CommandRearReset;
					break;
				case Keys.W:
					RearFunc = CommandRearFront;
					break;
				case Keys.S:
					RearFunc = CommandRearBack;
					break;

				case Keys.F:
					SteerFunc = CommandSteerReset;
					break;
				case Keys.D:
					SteerFunc = CommandSteerRight;
					break;
				case Keys.A:
					SteerFunc = CommandSteerLeft;
					break;

				case Keys.OemPeriod:
					PitchFunc = CommandPitchReset;
					break;
				case Keys.O:
					PitchFunc = CommandPitchUp;
					break;
				case Keys.L:
					PitchFunc = CommandPitchDown;
					break;

				case Keys.J:
					YawFunc = CommandYawReset;
					break;
				case Keys.K:
					YawFunc = CommandYawLeft;
					break;
				case Keys.OemSemicolon:
					YawFunc = CommandYawRight;
					break;
			}
		}
		private void CommonKeyUP(object sender, KeyEventArgs e)
		{
			switch (e.KeyCode)
			{
				case Keys.Q:
					if (SpecialFunc == CommandTurnOff)
						SpecialFunc = null;
					break;
				case Keys.E:
					if (RearFunc == CommandStopNow)
						RearFunc = null;
					break;

				case Keys.X:
					if (RearFunc == CommandRearReset)
						RearFunc = null;
					break;
				case Keys.W:
					if (RearFunc == CommandRearFront)
						RearFunc = null;
					break;
				case Keys.S:
					if (RearFunc == CommandRearBack)
						RearFunc = null;
					break;

				case Keys.F:
					if (SteerFunc == CommandSteerReset)
						SteerFunc = null;
					break;
				case Keys.D:
					if (SteerFunc == CommandSteerRight)
						SteerFunc = null;
					break;
				case Keys.A:
					if (SteerFunc == CommandSteerLeft)
						SteerFunc = null;
					break;

				case Keys.OemPeriod:
					if (PitchFunc == CommandPitchReset)
						PitchFunc = null;
					break;
				case Keys.O:
					if (PitchFunc == CommandPitchUp)
						PitchFunc = null;
					break;
				case Keys.L:
					if (PitchFunc == CommandPitchDown)
						PitchFunc = null;
					break;

				case Keys.J:
					if (YawFunc == CommandYawReset)
						YawFunc = null;
					break;
				case Keys.K:
					if (YawFunc == CommandYawLeft)
						YawFunc = null;
					break;
				case Keys.OemSemicolon:
					if (YawFunc == CommandYawRight)
						YawFunc = null;
					break;
			}
		}

		private void CommandTurnOff()
		{
			//NetMQMessage msg = new NetMQMessage();
			//msg.Append("COMMAND_PICAR", Encoding.ASCII);
			//msg.Append("TURN_OFF", Encoding.ASCII);

			//KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			//CommandQueue.Enqueue(val);
		}
		private void CommandStopNow()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("STOP", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}

		private void CommandRearReset()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}
		private void CommandRearFront()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetRear().CurValue + 100));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}
		private void CommandRearBack()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetRear().CurValue - 100));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}

		private void CommandSteerReset()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("STEER_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0.0f));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}
		private void CommandSteerRight()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("STEER_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetSteer().CurValue + 5.0f));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}
		private void CommandSteerLeft()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("STEER_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetSteer().CurValue - 5.0f));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}

		private void CommandPitchReset()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("PITCH_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0.0f));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}
		private void CommandPitchUp()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("PITCH_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraPitch().CurValue + 10.0f));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}
		private void CommandPitchDown()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("PITCH_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraPitch().CurValue - 10.0f));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}

		private void CommandYawReset()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("YAW_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0.0f));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}
		private void CommandYawRight()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("YAW_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraYaw().CurValue + 10.0f));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}
		private void CommandYawLeft()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("YAW_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraYaw().CurValue - 10.0f));

			KeyValuePair<bool, NetMQMessage> val = new(true, msg);
			CommandQueue.Enqueue(val);
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		public AutoDriveControlor()
		{
			InitializeComponent();
		}
		private void AutoDriveControlor_Load(object sender, EventArgs e)
		{
			Core.Init(CONNECTION_STRINGS.EXTERN_SUB, CONNECTION_STRINGS.LOCAL_PUB);
			NetMQTask = Task.Run(() => { NetMQThreadFunc(); });
			CameraViewTask = Task.Run(() => { UpdateCameraViewThreadFunc(); });
			CommandTask = Task.Run(() => { UpdateCommandThreadFunc(); });
		}
		private async void AutoDriveControlor_FormClosing(object sender, FormClosingEventArgs e)
		{
			Core.Release();
			NetMQProxy?.Stop();
			IsStopToUpdateCamera = true;
			IsStopToUpdateCommand = true;
			await NetMQTask;
			await CameraViewTask;
			await CommandTask;
		}
		private void AbsorbFocus(object sender, EventArgs e)
		{
			this.Focus();
		}
	}
}
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
		private Task NetMQTask;
		private NetMQPoller MainPoller = new();
		private NetMQTimer ConnectionTimer = new(TimeSpan.FromSeconds(1));
		private NetMQQueue<NetMQMessage> CommandQueue = new();
		private PublisherSocket CommandPub = new();
		private SubscriberSocket StateSub = new();

		private StateValues CurrentState = new();

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
				while (e.Queue.Count > 5)
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

		private bool IsStopToUpdateCamera = false;
		private Task CameraViewTask;
		private PictureBox ZoomInPb = null;
		private Font viewFontBig = new("Arial", 20, FontStyle.Bold);
		private Font viewFontSmall = new("Arial", 16, FontStyle.Bold);
		private SolidBrush blackBrush = new(Color.FromArgb(150, 0, 0, 0));
		private SolidBrush whiteBrush = new(Color.FromArgb(255, 255, 255, 255));
		private SolidBrush redBrush = new(Color.FromArgb(200, 0, 0));
		private SolidBrush greenBrush = new(Color.FromArgb(0, 200, 0));
		private SolidBrush blueBrush = new(Color.FromArgb(0, 0, 200));
		private Pen whitePen = new(Color.White, 10);

		private void UpdateCameraViewThreadFunc()
		{
			while (!IsStopToUpdateCamera)
			{
				PB_ViewTL.Invalidate();
				PB_ViewTR.Invalidate();

				Thread.Sleep(TimeSpan.FromMilliseconds(20));
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
			PictureBox targetPB = (PictureBox)sender;

			StateValues t_state = CurrentState.Clone();
			RectangleF zommImgRect = PB_CAL.CalZoomImagePictureBoxRectangle(targetPB.ClientRectangle, t_state.ImageFrame.Size);

			{
				string frameRateStr = t_state.FrameRate.ToString();
				PointF frameRateLoc = new(zommImgRect.Left + 10, zommImgRect.Top + 10);
				Size textSize = TextRenderer.MeasureText(frameRateStr, viewFontBig);
				RectangleF frameRateBgRect = new(frameRateLoc, textSize);
				frameRateBgRect.Width -= 5;

				e.Graphics.DrawImage(t_state.ImageFrame, zommImgRect);
				e.Graphics.FillRectangle(blackBrush, frameRateBgRect);
				e.Graphics.DrawString(frameRateStr, viewFontBig, whiteBrush, frameRateLoc);
			}

			{
				RectangleF speedBGRect = new(zommImgRect.Left + 10, zommImgRect.Bottom - 90, 30, 80);
				RectangleF speedFGRect = new(zommImgRect.Left + 10, zommImgRect.Bottom - 50, 30, 00);

				e.Graphics.FillRectangle(whiteBrush, speedBGRect);
				if (t_state.Rear.CurValue >= 0)
				{
					speedFGRect.Height = (float)Math.Abs(t_state.Rear.CurValue * 0.02);
					speedFGRect.Y -= speedFGRect.Height;
					e.Graphics.FillRectangle(blueBrush, speedFGRect);
				}
				else
				{
					speedFGRect.Height = (float)Math.Abs(t_state.Rear.CurValue * 0.02);
					e.Graphics.FillRectangle(redBrush, speedFGRect);
				}
			}

			{
				PointF arrowFrom = new(zommImgRect.Left + 80, zommImgRect.Bottom - 20);
				PointF arrowVec = new(0, -50);

				float t_sin = (float)Math.Sin(MATH.DEGREE_TO_RADIAN(t_state.Steer.CurValue));
				float t_cos = (float)Math.Cos(MATH.DEGREE_TO_RADIAN(t_state.Steer.CurValue));

				PointF newRotVec = new(
					arrowVec.X * t_cos - arrowVec.Y * t_sin,
					arrowVec.X * t_sin + arrowVec.Y * t_cos
				);
				PointF arrowTo = new(arrowFrom.X + newRotVec.X, arrowFrom.Y + newRotVec.Y);

				e.Graphics.DrawLine(whitePen, arrowFrom, arrowTo);
			}

			{
				float x_cos = (float)Math.Sin(MATH.DEGREE_TO_RADIAN(t_state.CameraYaw.CurValue)) * 40.0f;
				float y_cos = (float)Math.Sin(MATH.DEGREE_TO_RADIAN(t_state.CameraPitch.CurValue)) * 40.0f;

				RectangleF camBgRect = new(zommImgRect.Right - 110, zommImgRect.Bottom - 90, 100, 80);
				PointF camFgCir = new(zommImgRect.Right - 60 + x_cos, zommImgRect.Bottom - 50 - y_cos);
				RectangleF camFgRect = new(camFgCir.X - 5, camFgCir.Y - 5, 10, 10);

				e.Graphics.FillRectangle(whiteBrush, camBgRect);
				e.Graphics.FillEllipse(redBrush, camFgRect);
			}

			{
				string distanceStr = (t_state.SonicSensor * 0.001).ToString("0.000m");
				PointF distanceStrLoc = new(zommImgRect.X + zommImgRect.Width * 0.5f, zommImgRect.Bottom - 60);
				Size textSize = TextRenderer.MeasureText(distanceStr, viewFontSmall);
				distanceStrLoc.X -= textSize.Width * 0.5f;
				RectangleF distanceBgRect = new(distanceStrLoc, textSize);
				distanceBgRect.Width -= 5;

				e.Graphics.FillRectangle(blackBrush, distanceBgRect);
				e.Graphics.DrawString(distanceStr, viewFontSmall, whiteBrush, distanceStrLoc);
			}

			{
				PointF leftLoc = new(zommImgRect.X + zommImgRect.Width * 0.5f - 20, zommImgRect.Bottom - 20);
				PointF centerLoc = new(zommImgRect.X + zommImgRect.Width * 0.5f, zommImgRect.Bottom - 20);
				PointF rightLoc = new(zommImgRect.X + zommImgRect.Width * 0.5f + 20, zommImgRect.Bottom - 20);

				float multiLeftFloorVal = t_state.FloorSensor[0] / 1000.0f;
				float multiCenterFloorVal = t_state.FloorSensor[1] / 1000.0f;
				float multiRightFloorVal = t_state.FloorSensor[2] / 1000.0f;

				if (multiLeftFloorVal > 1.0f)
					multiLeftFloorVal = 1.0f;
				if (multiCenterFloorVal > 1.0f)
					multiCenterFloorVal = 1.0f;
				if (multiRightFloorVal > 1.0f)
					multiRightFloorVal = 1.0f;


				SolidBrush leftBrush = new(Color.FromArgb((int)(255 * multiLeftFloorVal), redBrush.Color));
				SolidBrush centerBrush = new(Color.FromArgb((int)(255 * multiCenterFloorVal), redBrush.Color));
				SolidBrush rightBrush = new(Color.FromArgb((int)(255 * multiRightFloorVal), redBrush.Color));

				RectangleF leftRect = new RectangleF(leftLoc.X - 5, leftLoc.Y - 5, 10, 10);
				RectangleF centerRect = new RectangleF(centerLoc.X - 5, centerLoc.Y - 5, 10, 10);
				RectangleF rightRect = new RectangleF(rightLoc.X - 5, rightLoc.Y - 5, 10, 10);

				e.Graphics.FillEllipse(leftBrush, leftRect);
				e.Graphics.FillEllipse(centerBrush, centerRect);
				e.Graphics.FillEllipse(rightBrush, rightRect);
			}
		}
		private void PB_ViewTR_Paint(object sender, PaintEventArgs e)
		{
			PictureBox targetPB = (PictureBox)sender;

			StateValues t_state = CurrentState.Clone();
			RectangleF zommImgRect = PB_CAL.CalZoomImagePictureBoxRectangle(targetPB.ClientRectangle, t_state.ImageFrame.Size);

			var img = t_state.ImageFrame;
			Rectangle rect = new(new Point(0, 0), img.Size);
			BitmapData imgData = img.LockBits(
				rect,
				ImageLockMode.ReadWrite,
				img.PixelFormat
			);

			AutoDriveCore.NormalizeRGB(img.Size.Width, img.Size.Height, imgData.Scan0);
			img.UnlockBits(imgData);

			e.Graphics.DrawImage(img, zommImgRect);
		}
		private void PB_ViewBL_Paint(object sender, PaintEventArgs e)
		{

		}
		private void PB_ViewBR_Paint(object sender, PaintEventArgs e)
		{

		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		private bool IsStopToUpdateCommand = false;
		private Task CommandTask;
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
			//CommandQueue.Enqueue(msg);
		}
		private void CommandStopNow()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("STOP", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0));
			CommandQueue.Enqueue(msg);
		}

		private void CommandRearReset()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0));
			CommandQueue.Enqueue(msg);
		}
		private void CommandRearFront()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetRear().CurValue + 100));
			CommandQueue.Enqueue(msg);
		}
		private void CommandRearBack()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("REAR_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetRear().CurValue - 100));
			CommandQueue.Enqueue(msg);
		}

		private void CommandSteerReset()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("STEER_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0.0f));
			CommandQueue.Enqueue(msg);
		}
		private void CommandSteerRight()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("STEER_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetSteer().CurValue + 5.0f));
			CommandQueue.Enqueue(msg);
		}
		private void CommandSteerLeft()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_MOVE_MOTOR", Encoding.ASCII);
			msg.Append("STEER_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetSteer().CurValue - 5.0f));
			CommandQueue.Enqueue(msg);
		}

		private void CommandPitchReset()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("PITCH_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0.0f));
			CommandQueue.Enqueue(msg);
		}
		private void CommandPitchUp()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("PITCH_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraPitch().CurValue + 10.0f));
			CommandQueue.Enqueue(msg);
		}
		private void CommandPitchDown()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("PITCH_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraPitch().CurValue - 10.0f));
			CommandQueue.Enqueue(msg);
		}

		private void CommandYawReset()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("YAW_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(0.0f));
			CommandQueue.Enqueue(msg);
		}
		private void CommandYawRight()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("YAW_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraYaw().CurValue + 10.0f));
			CommandQueue.Enqueue(msg);
		}
		private void CommandYawLeft()
		{
			NetMQMessage msg = new NetMQMessage();
			msg.Append("COMMAND_CAMERA_MOTOR", Encoding.ASCII);
			msg.Append("YAW_MOTOR", Encoding.ASCII);
			msg.Append("VALUE", Encoding.ASCII);
			msg.Append(BitConverter.GetBytes(CurrentState.GetCameraYaw().CurValue - 10.0f));
			CommandQueue.Enqueue(msg);
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		public AutoDriveControlor()
		{
			InitializeComponent();
		}
		private void AutoDriveControlor_Load(object sender, EventArgs e)
		{
			NetMQTask = Task.Run(() => { NetMQThreadFunc(); });
			CameraViewTask = Task.Run(() => { UpdateCameraViewThreadFunc(); });
			CommandTask = Task.Run(() => { UpdateCommandThreadFunc(); });
		}
		private async void AutoDriveControlor_FormClosing(object sender, FormClosingEventArgs e)
		{
			MainPoller.Stop();
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

		private void BTN_TSET_Click(object sender, EventArgs e)
		{
			var img = CurrentState.GetImageFrame();
			Rectangle rect = new(new Point(0, 0), img.Size);
			BitmapData imgData = img.LockBits(
				rect,
				ImageLockMode.ReadOnly,
				img.PixelFormat
			);

			IntPtr dataPtr = imgData.Scan0;
			AutoDriveCore.TEST(img.Size.Width, img.Size.Height, dataPtr);
		}
	}
}
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
		private bool IsStopToUpdateCamera = false;
		private Task? CameraViewTask = null;
		private object ImageLock = new();
		private bool IsZoomIn = false;
		private Bitmap? StateImage = null;

		private void UpdateCameraViewThreadFunc()
		{
			while (!IsStopToUpdateCamera)
			{
				DateTime start = DateTime.Now;

				Core.ImageData? stateData = Core.GetStateImage();
				lock (ImageLock)
				{
					Bitmap? t_stateImage = StateImage;
					StateImage = stateData?.ToBitmap();
					t_stateImage?.Dispose();
				}
				PB_ViewImage.Invalidate();

				if ((TimeSpan.FromMilliseconds(33) - (DateTime.Now - start)).Milliseconds > 0)
					Thread.Sleep(TimeSpan.FromMilliseconds(33) - (DateTime.Now - start));
			}
		}

		private void PB_ViewImage_Paint(object sender, PaintEventArgs e)
		{
			lock (ImageLock)
			{
				if (StateImage == null) return;
				PictureBox targetPB = (PictureBox)sender;
				RectangleF zommImgRect = PB_CAL.CalZoomImagePictureBoxRectangle(targetPB.ClientRectangle, StateImage.Size);
				e.Graphics.DrawImage(StateImage, zommImgRect);
			}
		}
		private void PB_ViewImage_DoubleClick(object sender, EventArgs e)
		{
			if (IsZoomIn)
			{
				IsZoomIn = false;
				PB_ViewImage.Size = new Size(320, 240);
			}
			else
			{
				IsZoomIn = true;
				PB_ViewImage.Size = PB_ViewVisualizer.Size;
			}
		}
		private void PB_ViewVisualizer_Resize(object sender, EventArgs e)
		{
			Core.ExecuteEvent("RESIZE_VISUALIZER");
			if (IsZoomIn)
				PB_ViewImage.Size = PB_ViewVisualizer.Size;
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

				case Keys.Space:
					Thread th = new(new ThreadStart(() => { Core.ExecuteEvent("PUSH_IMAGE_PC"); }));
					th.Start();
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

		private void CommandTurnOff() { Core.TurnOff(); }
		private void CommandStopNow() { Core.StopMove(); }

		private void CommandRearReset() { Core.ChangeRearValue(0); }
		private void CommandRearFront() { Core.ChangeRearValue(100); }
		private void CommandRearBack() { Core.ChangeRearValue(-100); }

		private void CommandSteerReset() { Core.ChangeSteerValue(0.0f); }
		private void CommandSteerRight() { Core.ChangeSteerValue(5.0f); }
		private void CommandSteerLeft() { Core.ChangeSteerValue(-5.0f); }

		private void CommandPitchReset() { Core.ChangeCameraPitchValue(0.0f); }
		private void CommandPitchUp() { Core.ChangeCameraPitchValue(10.0f); }
		private void CommandPitchDown() { Core.ChangeCameraPitchValue(-10.0f); }

		private void CommandYawReset() { Core.ChangeCameraYawValue(0.0f); }
		private void CommandYawRight() { Core.ChangeCameraYawValue(10.0f); }
		private void CommandYawLeft() { Core.ChangeCameraYawValue(-10.0f); }

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		public AutoDriveControlor()
		{
			InitializeComponent();
		}
		private void AutoDriveControlor_Load(object sender, EventArgs e)
		{
			Core.Init(CONNECTION_STRINGS.EXTERN_SUB, CONNECTION_STRINGS.EXTERN_PUB, PB_ViewVisualizer.Handle);
			CameraViewTask = Task.Run(() => { UpdateCameraViewThreadFunc(); });
			CommandTask = Task.Run(() => { UpdateCommandThreadFunc(); });
		}
		private void AutoDriveControlor_FormClosing(object sender, FormClosingEventArgs e)
		{
			IsStopToUpdateCamera = true;
			IsStopToUpdateCommand = true;
			CameraViewTask?.Wait();
			CommandTask?.Wait();
			Core.Release();
		}
		private void AbsorbFocus(object sender, EventArgs e)
		{
			this.Focus();
		}

		private void BTN_FocusMainForm_Click(object sender, EventArgs e)
		{
			this.Focus();
		}
	}
}
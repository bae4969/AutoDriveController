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
		private PictureBox? ZoomInPb = null;
		private Bitmap? StateImage = null;
		private Bitmap? FilterImage = null;

		private void UpdateCameraViewThreadFunc()
		{
			while (!IsStopToUpdateCamera)
			{
				DateTime start = DateTime.Now;

				Core.ImageData? stateData = Core.GetStateImage();
				Core.ImageData? filterData = Core.GetFilterImage();
				lock (ImageLock)
				{
					Bitmap? t_stateImage = StateImage;
					Bitmap? t_filterImage = FilterImage;
					StateImage = stateData?.ToBitmap();
					FilterImage = filterData?.ToBitmap();
					t_stateImage?.Dispose();
					t_filterImage?.Dispose();
				}
				PB_ViewTL.Invalidate();
				PB_ViewTR.Invalidate();

				if ((TimeSpan.FromMilliseconds(33) - (DateTime.Now - start)).Milliseconds > 0)
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
			lock (ImageLock)
			{
				if (StateImage == null) return;
				PictureBox targetPB = (PictureBox)sender;
				RectangleF zommImgRect = PB_CAL.CalZoomImagePictureBoxRectangle(targetPB.ClientRectangle, StateImage.Size);
				e.Graphics.DrawImage(StateImage, zommImgRect);
			}
		}
		private void PB_ViewTR_Paint(object sender, PaintEventArgs e)
		{
			lock (ImageLock)
			{
				if (FilterImage == null) return;
				PictureBox targetPB = (PictureBox)sender;
				RectangleF zommImgRect = PB_CAL.CalZoomImagePictureBoxRectangle(targetPB.ClientRectangle, FilterImage.Size);
				e.Graphics.DrawImage(FilterImage, zommImgRect);
			}
		}
		private void PB_ViewBL_Paint(object sender, PaintEventArgs e)
		{
			lock (ImageLock)
			{

			}
		}
		private void PB_ViewBR_Paint(object sender, PaintEventArgs e)
		{
			lock (ImageLock)
			{

			}
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

		private void CommandYawReset() { Core.ChangeCameraPitchYaw(0.0f); }
		private void CommandYawRight() { Core.ChangeCameraPitchYaw(10.0f); }
		private void CommandYawLeft() { Core.ChangeCameraPitchYaw(-10.0f); }

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		public AutoDriveControlor()
		{
			InitializeComponent();
		}
		private void AutoDriveControlor_Load(object sender, EventArgs e)
		{
			Core.Init(CONNECTION_STRINGS.EXTERN_SUB, CONNECTION_STRINGS.EXTERN_PUB);
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

		private void BTN_TSET_Click(object sender, EventArgs e)
		{
			lock (ImageLock)
			{
				StateImage?.Save("../../1_state.png");
				FilterImage?.Save("../../2_filter.png");
			}
		}
	}
}
namespace AutoDriveControlor.Forms
{
	partial class AutoDriveControlor
	{
		/// <summary>
		///  Required designer variable.
		/// </summary>
		private System.ComponentModel.IContainer components = null;

		/// <summary>
		///  Clean up any resources being used.
		/// </summary>
		/// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
		protected override void Dispose(bool disposing)
		{
			if (disposing && (components != null))
			{
				components.Dispose();
			}
			base.Dispose(disposing);
		}

		#region Windows Form Designer generated code

		/// <summary>
		///  Required method for Designer support - do not modify
		///  the contents of this method with the code editor.
		/// </summary>
		private void InitializeComponent()
		{
			PB_CameraView = new PictureBox();
			BTN_SteerLeft = new Button();
			BTN_SteerRight = new Button();
			BTN_RearFront = new Button();
			BTN_RearBack = new Button();
			BTN_PitchDown = new Button();
			BTN_PitchUp = new Button();
			BTN_YawRight = new Button();
			BTN_YawLeft = new Button();
			BTN_Stop = new Button();
			BTN_ResetSteer = new Button();
			BTN_ResetRear = new Button();
			BTN_ResetYaw = new Button();
			BTN_ResetPitch = new Button();
			BTN_TurnOff = new Button();
			((System.ComponentModel.ISupportInitialize)PB_CameraView).BeginInit();
			SuspendLayout();
			// 
			// PB_CameraView
			// 
			PB_CameraView.Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
			PB_CameraView.Location = new Point(12, 12);
			PB_CameraView.Name = "PB_CameraView";
			PB_CameraView.Size = new Size(912, 737);
			PB_CameraView.SizeMode = PictureBoxSizeMode.Zoom;
			PB_CameraView.TabIndex = 0;
			PB_CameraView.TabStop = false;
			// 
			// BTN_SteerLeft
			// 
			BTN_SteerLeft.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_SteerLeft.Location = new Point(998, 525);
			BTN_SteerLeft.Name = "BTN_SteerLeft";
			BTN_SteerLeft.Size = new Size(50, 40);
			BTN_SteerLeft.TabIndex = 1;
			BTN_SteerLeft.Text = "◀";
			BTN_SteerLeft.UseVisualStyleBackColor = true;
			BTN_SteerLeft.Click += BTN_SteerLeft_Click;
			// 
			// BTN_SteerRight
			// 
			BTN_SteerRight.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_SteerRight.Location = new Point(1110, 525);
			BTN_SteerRight.Margin = new Padding(3, 3, 15, 3);
			BTN_SteerRight.Name = "BTN_SteerRight";
			BTN_SteerRight.Size = new Size(50, 40);
			BTN_SteerRight.TabIndex = 2;
			BTN_SteerRight.Text = "▶";
			BTN_SteerRight.UseVisualStyleBackColor = true;
			BTN_SteerRight.Click += BTN_SteerRight_Click;
			// 
			// BTN_RearFront
			// 
			BTN_RearFront.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_RearFront.Location = new Point(1054, 479);
			BTN_RearFront.Name = "BTN_RearFront";
			BTN_RearFront.Size = new Size(50, 40);
			BTN_RearFront.TabIndex = 3;
			BTN_RearFront.Text = "▲";
			BTN_RearFront.UseVisualStyleBackColor = true;
			BTN_RearFront.Click += BTN_RearFront_Click;
			// 
			// BTN_RearBack
			// 
			BTN_RearBack.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_RearBack.Location = new Point(1054, 525);
			BTN_RearBack.Margin = new Padding(3, 3, 3, 20);
			BTN_RearBack.Name = "BTN_RearBack";
			BTN_RearBack.Size = new Size(50, 40);
			BTN_RearBack.TabIndex = 4;
			BTN_RearBack.Text = "▼";
			BTN_RearBack.UseVisualStyleBackColor = true;
			BTN_RearBack.Click += BTN_RearBack_Click;
			// 
			// BTN_PitchDown
			// 
			BTN_PitchDown.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_PitchDown.Location = new Point(1054, 697);
			BTN_PitchDown.Margin = new Padding(3, 3, 3, 15);
			BTN_PitchDown.Name = "BTN_PitchDown";
			BTN_PitchDown.Size = new Size(50, 40);
			BTN_PitchDown.TabIndex = 8;
			BTN_PitchDown.Text = "▼";
			BTN_PitchDown.UseVisualStyleBackColor = true;
			BTN_PitchDown.Click += BTN_PitchDown_Click;
			// 
			// BTN_PitchUp
			// 
			BTN_PitchUp.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_PitchUp.Location = new Point(1054, 651);
			BTN_PitchUp.Name = "BTN_PitchUp";
			BTN_PitchUp.Size = new Size(50, 40);
			BTN_PitchUp.TabIndex = 7;
			BTN_PitchUp.Text = "▲";
			BTN_PitchUp.UseVisualStyleBackColor = true;
			BTN_PitchUp.Click += BTN_PitchUp_Click;
			// 
			// BTN_YawRight
			// 
			BTN_YawRight.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_YawRight.Location = new Point(1110, 697);
			BTN_YawRight.Margin = new Padding(3, 3, 15, 15);
			BTN_YawRight.Name = "BTN_YawRight";
			BTN_YawRight.Size = new Size(50, 40);
			BTN_YawRight.TabIndex = 6;
			BTN_YawRight.Text = "▶";
			BTN_YawRight.UseVisualStyleBackColor = true;
			BTN_YawRight.Click += BTN_YawRight_Click;
			// 
			// BTN_YawLeft
			// 
			BTN_YawLeft.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_YawLeft.Location = new Point(998, 697);
			BTN_YawLeft.Margin = new Padding(3, 3, 3, 15);
			BTN_YawLeft.Name = "BTN_YawLeft";
			BTN_YawLeft.Size = new Size(50, 40);
			BTN_YawLeft.TabIndex = 5;
			BTN_YawLeft.Text = "◀";
			BTN_YawLeft.UseVisualStyleBackColor = true;
			BTN_YawLeft.Click += BTN_YawLeft_Click;
			// 
			// BTN_Stop
			// 
			BTN_Stop.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_Stop.Location = new Point(998, 370);
			BTN_Stop.Margin = new Padding(3, 3, 15, 3);
			BTN_Stop.Name = "BTN_Stop";
			BTN_Stop.Size = new Size(162, 40);
			BTN_Stop.TabIndex = 9;
			BTN_Stop.Text = "Stop Now";
			BTN_Stop.UseVisualStyleBackColor = true;
			BTN_Stop.Click += BTN_Stop_Click;
			// 
			// BTN_ResetSteer
			// 
			BTN_ResetSteer.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_ResetSteer.Location = new Point(942, 525);
			BTN_ResetSteer.Margin = new Padding(15, 3, 3, 3);
			BTN_ResetSteer.Name = "BTN_ResetSteer";
			BTN_ResetSteer.Size = new Size(50, 40);
			BTN_ResetSteer.TabIndex = 10;
			BTN_ResetSteer.Text = "Reset\r\nSteer";
			BTN_ResetSteer.UseVisualStyleBackColor = true;
			BTN_ResetSteer.Click += BTN_ResetSteer_Click;
			// 
			// BTN_ResetRear
			// 
			BTN_ResetRear.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_ResetRear.Location = new Point(1054, 433);
			BTN_ResetRear.Margin = new Padding(3, 20, 3, 3);
			BTN_ResetRear.Name = "BTN_ResetRear";
			BTN_ResetRear.Size = new Size(50, 40);
			BTN_ResetRear.TabIndex = 11;
			BTN_ResetRear.Text = "Reset\r\nRear";
			BTN_ResetRear.UseVisualStyleBackColor = true;
			BTN_ResetRear.Click += BTN_ResetRear_Click;
			// 
			// BTN_ResetYaw
			// 
			BTN_ResetYaw.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_ResetYaw.Location = new Point(942, 697);
			BTN_ResetYaw.Margin = new Padding(15, 3, 3, 15);
			BTN_ResetYaw.Name = "BTN_ResetYaw";
			BTN_ResetYaw.Size = new Size(50, 40);
			BTN_ResetYaw.TabIndex = 12;
			BTN_ResetYaw.Text = "Reset\r\nYaw";
			BTN_ResetYaw.UseVisualStyleBackColor = true;
			BTN_ResetYaw.Click += BTN_ResetYaw_Click;
			// 
			// BTN_ResetPitch
			// 
			BTN_ResetPitch.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_ResetPitch.Location = new Point(1054, 605);
			BTN_ResetPitch.Margin = new Padding(3, 20, 3, 3);
			BTN_ResetPitch.Name = "BTN_ResetPitch";
			BTN_ResetPitch.Size = new Size(50, 40);
			BTN_ResetPitch.TabIndex = 13;
			BTN_ResetPitch.Text = "Reset\r\nPitch";
			BTN_ResetPitch.UseVisualStyleBackColor = true;
			BTN_ResetPitch.Click += BTN_ResetPitch_Click;
			// 
			// BTN_TurnOff
			// 
			BTN_TurnOff.Anchor = AnchorStyles.Top | AnchorStyles.Right;
			BTN_TurnOff.Location = new Point(998, 24);
			BTN_TurnOff.Margin = new Padding(15);
			BTN_TurnOff.Name = "BTN_TurnOff";
			BTN_TurnOff.Size = new Size(162, 40);
			BTN_TurnOff.TabIndex = 14;
			BTN_TurnOff.Text = "Turn OFF";
			BTN_TurnOff.UseVisualStyleBackColor = true;
			BTN_TurnOff.Click += BTN_TurnOff_Click;
			// 
			// AutoDriveControlor
			// 
			AutoScaleDimensions = new SizeF(7F, 15F);
			AutoScaleMode = AutoScaleMode.Font;
			ClientSize = new Size(1184, 761);
			Controls.Add(BTN_TurnOff);
			Controls.Add(BTN_ResetPitch);
			Controls.Add(BTN_ResetYaw);
			Controls.Add(BTN_ResetRear);
			Controls.Add(BTN_ResetSteer);
			Controls.Add(BTN_Stop);
			Controls.Add(BTN_PitchDown);
			Controls.Add(BTN_PitchUp);
			Controls.Add(BTN_YawRight);
			Controls.Add(BTN_YawLeft);
			Controls.Add(BTN_RearBack);
			Controls.Add(BTN_RearFront);
			Controls.Add(BTN_SteerRight);
			Controls.Add(BTN_SteerLeft);
			Controls.Add(PB_CameraView);
			Font = new Font("Arial", 9F, FontStyle.Regular, GraphicsUnit.Point);
			Name = "AutoDriveControlor";
			Text = "AutoDrive Controlor";
			FormClosing += AutoDriveControlor_FormClosing;
			Load += AutoDriveControlor_Load;
			((System.ComponentModel.ISupportInitialize)PB_CameraView).EndInit();
			ResumeLayout(false);
		}

		#endregion

		public PictureBox PB_CameraView;
		private Button BTN_SteerLeft;
		private Button BTN_SteerRight;
		private Button BTN_RearFront;
		private Button BTN_RearBack;
		private Button BTN_PitchDown;
		private Button BTN_PitchUp;
		private Button BTN_YawRight;
		private Button BTN_YawLeft;
		private Button BTN_Stop;
		private Button BTN_ResetSteer;
		private Button BTN_ResetRear;
		private Button BTN_ResetYaw;
		private Button BTN_ResetPitch;
		private Button BTN_TurnOff;
	}
}
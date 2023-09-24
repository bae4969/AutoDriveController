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
			PB_ViewImage = new PictureBox();
			PB_ViewVisualizer = new PictureBox();
			BTN_FocusMainForm = new Button();
			((System.ComponentModel.ISupportInitialize)PB_ViewImage).BeginInit();
			((System.ComponentModel.ISupportInitialize)PB_ViewVisualizer).BeginInit();
			SuspendLayout();
			// 
			// PB_ViewImage
			// 
			PB_ViewImage.BackColor = SystemColors.ControlDark;
			PB_ViewImage.Location = new Point(0, 0);
			PB_ViewImage.Name = "PB_ViewImage";
			PB_ViewImage.Size = new Size(320, 240);
			PB_ViewImage.SizeMode = PictureBoxSizeMode.Zoom;
			PB_ViewImage.TabIndex = 16;
			PB_ViewImage.TabStop = false;
			PB_ViewImage.Paint += PB_ViewImage_Paint;
			PB_ViewImage.DoubleClick += PB_ViewImage_DoubleClick;
			PB_ViewImage.Validated += AbsorbFocus;
			// 
			// PB_ViewVisualizer
			// 
			PB_ViewVisualizer.BackColor = SystemColors.ControlDark;
			PB_ViewVisualizer.Dock = DockStyle.Fill;
			PB_ViewVisualizer.Location = new Point(0, 0);
			PB_ViewVisualizer.Name = "PB_ViewVisualizer";
			PB_ViewVisualizer.Size = new Size(1184, 761);
			PB_ViewVisualizer.SizeMode = PictureBoxSizeMode.Zoom;
			PB_ViewVisualizer.TabIndex = 17;
			PB_ViewVisualizer.TabStop = false;
			PB_ViewVisualizer.Resize += PB_ViewVisualizer_Resize;
			// 
			// BTN_FocusMainForm
			// 
			BTN_FocusMainForm.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
			BTN_FocusMainForm.Font = new Font("Arial Black", 14.25F, FontStyle.Bold, GraphicsUnit.Point);
			BTN_FocusMainForm.Location = new Point(979, 720);
			BTN_FocusMainForm.Name = "BTN_FocusMainForm";
			BTN_FocusMainForm.Size = new Size(205, 41);
			BTN_FocusMainForm.TabIndex = 2;
			BTN_FocusMainForm.Text = "To control Mode";
			BTN_FocusMainForm.UseVisualStyleBackColor = true;
			BTN_FocusMainForm.Click += BTN_FocusMainForm_Click;
			// 
			// AutoDriveControlor
			// 
			AutoScaleDimensions = new SizeF(7F, 15F);
			AutoScaleMode = AutoScaleMode.Font;
			ClientSize = new Size(1184, 761);
			Controls.Add(BTN_FocusMainForm);
			Controls.Add(PB_ViewImage);
			Controls.Add(PB_ViewVisualizer);
			DoubleBuffered = true;
			Font = new Font("Arial", 9F, FontStyle.Regular, GraphicsUnit.Point);
			KeyPreview = true;
			Name = "AutoDriveControlor";
			Text = "AutoDrive Controlor";
			FormClosing += AutoDriveControlor_FormClosing;
			Load += AutoDriveControlor_Load;
			KeyDown += CommonKeyDown;
			KeyUp += CommonKeyUP;
			((System.ComponentModel.ISupportInitialize)PB_ViewImage).EndInit();
			((System.ComponentModel.ISupportInitialize)PB_ViewVisualizer).EndInit();
			ResumeLayout(false);
		}

		#endregion
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
		private PictureBox PB_ViewImage;
		private TableLayoutPanel tableLayoutPanel1;
		private TableLayoutPanel tableLayoutPanel2;
		private Button BTN_FocusMainForm;
		private PictureBox PB_ViewVisualizer;
	}
}
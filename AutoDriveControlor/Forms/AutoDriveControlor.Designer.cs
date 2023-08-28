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
			PB_ViewTL = new PictureBox();
			PB_ViewTR = new PictureBox();
			PB_ViewBL = new PictureBox();
			PB_ViewBR = new PictureBox();
			TLP_ViewLayoutTable = new TableLayoutPanel();
			SC_ViewAndControl = new SplitContainer();
			BTN_TSET = new Button();
			((System.ComponentModel.ISupportInitialize)PB_ViewTL).BeginInit();
			((System.ComponentModel.ISupportInitialize)PB_ViewTR).BeginInit();
			((System.ComponentModel.ISupportInitialize)PB_ViewBL).BeginInit();
			((System.ComponentModel.ISupportInitialize)PB_ViewBR).BeginInit();
			TLP_ViewLayoutTable.SuspendLayout();
			((System.ComponentModel.ISupportInitialize)SC_ViewAndControl).BeginInit();
			SC_ViewAndControl.Panel1.SuspendLayout();
			SC_ViewAndControl.Panel2.SuspendLayout();
			SC_ViewAndControl.SuspendLayout();
			SuspendLayout();
			// 
			// PB_ViewTL
			// 
			PB_ViewTL.BackColor = SystemColors.ControlDark;
			PB_ViewTL.Dock = DockStyle.Fill;
			PB_ViewTL.Location = new Point(3, 3);
			PB_ViewTL.Name = "PB_ViewTL";
			PB_ViewTL.Size = new Size(478, 374);
			PB_ViewTL.SizeMode = PictureBoxSizeMode.Zoom;
			PB_ViewTL.TabIndex = 16;
			PB_ViewTL.TabStop = false;
			PB_ViewTL.Paint += PB_ViewTL_Paint;
			PB_ViewTL.DoubleClick += PB_DoubleClickEvent;
			PB_ViewTL.Validated += AbsorbFocus;
			// 
			// PB_ViewTR
			// 
			PB_ViewTR.BackColor = SystemColors.ControlDark;
			PB_ViewTR.Dock = DockStyle.Fill;
			PB_ViewTR.Location = new Point(487, 3);
			PB_ViewTR.Name = "PB_ViewTR";
			PB_ViewTR.Size = new Size(479, 374);
			PB_ViewTR.SizeMode = PictureBoxSizeMode.Zoom;
			PB_ViewTR.TabIndex = 17;
			PB_ViewTR.TabStop = false;
			PB_ViewTR.Paint += PB_ViewTR_Paint;
			PB_ViewTR.DoubleClick += PB_DoubleClickEvent;
			PB_ViewTR.Validated += AbsorbFocus;
			// 
			// PB_ViewBL
			// 
			PB_ViewBL.BackColor = SystemColors.ControlDark;
			PB_ViewBL.Dock = DockStyle.Fill;
			PB_ViewBL.Location = new Point(3, 383);
			PB_ViewBL.Name = "PB_ViewBL";
			PB_ViewBL.Size = new Size(478, 375);
			PB_ViewBL.SizeMode = PictureBoxSizeMode.Zoom;
			PB_ViewBL.TabIndex = 18;
			PB_ViewBL.TabStop = false;
			PB_ViewBL.Paint += PB_ViewBL_Paint;
			PB_ViewBL.DoubleClick += PB_DoubleClickEvent;
			PB_ViewBL.Validated += AbsorbFocus;
			// 
			// PB_ViewBR
			// 
			PB_ViewBR.BackColor = SystemColors.ControlDark;
			PB_ViewBR.Dock = DockStyle.Fill;
			PB_ViewBR.Location = new Point(487, 383);
			PB_ViewBR.Name = "PB_ViewBR";
			PB_ViewBR.Size = new Size(479, 375);
			PB_ViewBR.SizeMode = PictureBoxSizeMode.Zoom;
			PB_ViewBR.TabIndex = 18;
			PB_ViewBR.TabStop = false;
			PB_ViewBR.Paint += PB_ViewBR_Paint;
			PB_ViewBR.DoubleClick += PB_DoubleClickEvent;
			PB_ViewBR.Validated += AbsorbFocus;
			// 
			// TLP_ViewLayoutTable
			// 
			TLP_ViewLayoutTable.ColumnCount = 2;
			TLP_ViewLayoutTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
			TLP_ViewLayoutTable.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
			TLP_ViewLayoutTable.Controls.Add(PB_ViewBR, 1, 1);
			TLP_ViewLayoutTable.Controls.Add(PB_ViewBL, 0, 1);
			TLP_ViewLayoutTable.Controls.Add(PB_ViewTR, 1, 0);
			TLP_ViewLayoutTable.Controls.Add(PB_ViewTL, 0, 0);
			TLP_ViewLayoutTable.Dock = DockStyle.Fill;
			TLP_ViewLayoutTable.Location = new Point(0, 0);
			TLP_ViewLayoutTable.Name = "TLP_ViewLayoutTable";
			TLP_ViewLayoutTable.RowCount = 2;
			TLP_ViewLayoutTable.RowStyles.Add(new RowStyle(SizeType.Percent, 50F));
			TLP_ViewLayoutTable.RowStyles.Add(new RowStyle(SizeType.Percent, 50F));
			TLP_ViewLayoutTable.Size = new Size(969, 761);
			TLP_ViewLayoutTable.TabIndex = 18;
			// 
			// SC_ViewAndControl
			// 
			SC_ViewAndControl.Dock = DockStyle.Fill;
			SC_ViewAndControl.FixedPanel = FixedPanel.Panel2;
			SC_ViewAndControl.IsSplitterFixed = true;
			SC_ViewAndControl.Location = new Point(0, 0);
			SC_ViewAndControl.Name = "SC_ViewAndControl";
			// 
			// SC_ViewAndControl.Panel1
			// 
			SC_ViewAndControl.Panel1.Controls.Add(TLP_ViewLayoutTable);
			// 
			// SC_ViewAndControl.Panel2
			// 
			SC_ViewAndControl.Panel2.Controls.Add(BTN_TSET);
			SC_ViewAndControl.Size = new Size(1184, 761);
			SC_ViewAndControl.SplitterDistance = 969;
			SC_ViewAndControl.TabIndex = 19;
			// 
			// BTN_TSET
			// 
			BTN_TSET.Location = new Point(28, 33);
			BTN_TSET.Name = "BTN_TSET";
			BTN_TSET.Size = new Size(75, 23);
			BTN_TSET.TabIndex = 0;
			BTN_TSET.Text = "test";
			BTN_TSET.UseVisualStyleBackColor = true;
			// 
			// AutoDriveControlor
			// 
			AutoScaleDimensions = new SizeF(7F, 15F);
			AutoScaleMode = AutoScaleMode.Font;
			ClientSize = new Size(1184, 761);
			Controls.Add(SC_ViewAndControl);
			DoubleBuffered = true;
			Font = new Font("Arial", 9F, FontStyle.Regular, GraphicsUnit.Point);
			KeyPreview = true;
			Name = "AutoDriveControlor";
			Text = "AutoDrive Controlor";
			FormClosing += AutoDriveControlor_FormClosing;
			Load += AutoDriveControlor_Load;
			KeyDown += CommonKeyDown;
			KeyUp += CommonKeyUP;
			((System.ComponentModel.ISupportInitialize)PB_ViewTL).EndInit();
			((System.ComponentModel.ISupportInitialize)PB_ViewTR).EndInit();
			((System.ComponentModel.ISupportInitialize)PB_ViewBL).EndInit();
			((System.ComponentModel.ISupportInitialize)PB_ViewBR).EndInit();
			TLP_ViewLayoutTable.ResumeLayout(false);
			SC_ViewAndControl.Panel1.ResumeLayout(false);
			SC_ViewAndControl.Panel2.ResumeLayout(false);
			((System.ComponentModel.ISupportInitialize)SC_ViewAndControl).EndInit();
			SC_ViewAndControl.ResumeLayout(false);
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
		private PictureBox PB_ViewTL;
		private PictureBox PB_ViewTR;
		private PictureBox PB_ViewBL;
		private PictureBox PB_ViewBR;
		private TableLayoutPanel tableLayoutPanel1;
		private TableLayoutPanel tableLayoutPanel2;
		private TableLayoutPanel TLP_ViewLayoutTable;
		private SplitContainer SC_ViewAndControl;
		private Button BTN_TSET;
	}
}
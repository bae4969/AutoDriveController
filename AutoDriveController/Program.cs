

namespace AutoDriveControlor
{
	internal static class Program
	{
		[STAThread]
		static void Main()
		{
			// 우선은 한개만 실행 가능하도록 설정
			// 여러개 실행하려면 대역폭 문제로 서버쪽 프록시도 필요할 것으로 보임
			_ = new Mutex(true, "AUTO_DRIVE_CONTROLOR_INIT_MUTEX", out bool isNew);
			if (!isNew)
			{
				MessageBox.Show(
					"Already Running",
					"Error",
					MessageBoxButtons.OK,
					MessageBoxIcon.Error
				);
				return;
			}

			Environment.SetEnvironmentVariable("Path", "./bin;D:\\code\\AutoDriveController\\temp;");

			ApplicationConfiguration.Initialize();
			Application.Run(new Forms.AutoDriveControlor());
		}
	}
}
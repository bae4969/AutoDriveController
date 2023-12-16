

namespace AutoDriveControlor
{
	internal static class Program
	{
		[STAThread]
		static void Main()
		{
			// �켱�� �Ѱ��� ���� �����ϵ��� ����
			// ������ �����Ϸ��� �뿪�� ������ ������ ���Ͻõ� �ʿ��� ������ ����
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
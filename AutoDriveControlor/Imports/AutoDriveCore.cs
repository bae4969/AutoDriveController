using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace AutoDriveControlor.Imports
{
	internal static class AutoDriveCore
	{
#if DEBUG
		private const string DLL_FILE_NAME = "AutoDriveCore_d.dll";
#else
		private const string DLL_FILE_NAME = "AutoDriveCore.dll";
#endif

		[DllImport(DLL_FILE_NAME)]
		public static extern int TEST(int w, int h, IntPtr data);

		[DllImport(DLL_FILE_NAME)]
		public static extern int NormalizeRGB(int w, int h, IntPtr data);
	}
}

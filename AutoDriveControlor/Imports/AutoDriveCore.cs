using System;
using System.Collections.Generic;
using System.Drawing.Imaging;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using static System.Runtime.InteropServices.JavaScript.JSType;

namespace AutoDriveControlor.Imports
{
	internal static class Core
	{
#if DEBUG
		private const string DLL_FILE_NAME = "AutoDriveCore_d.dll";
#else
		private const string DLL_FILE_NAME = "AutoDriveCore.dll";
#endif


		[StructLayout(LayoutKind.Sequential)]
		public struct ImageData
		{
			[MarshalAs(UnmanagedType.I4)] int W;
			[MarshalAs(UnmanagedType.I4)] int H;
			[MarshalAs(UnmanagedType.I4)] int Ch;
			[MarshalAs(UnmanagedType.I4)] int Step;
			IntPtr Data;

			public unsafe Bitmap? ToBitmap()
			{
				PixelFormat format;
				if (Ch == 1 && Step == 1)
					format = PixelFormat.Format8bppIndexed;
				else if (Ch == 1 && Step == 2)
					format = PixelFormat.Format16bppGrayScale;
				else if (Ch == 3 && Step == 1)
					format = PixelFormat.Format24bppRgb;
				else if (Ch == 4 && Step == 1)
					format = PixelFormat.Format32bppArgb;
				else
					return null;

				Bitmap bitmap = new(W, H, format);
				Rectangle rect = new(0, 0, W, H);
				BitmapData imgData = bitmap.LockBits(
					rect,
					ImageLockMode.WriteOnly,
					bitmap.PixelFormat
				);
				int totMemSize = W * H * Ch * Step;
				Buffer.MemoryCopy((void*)Data, (void*)imgData.Scan0, totMemSize, totMemSize);
				bitmap.UnlockBits(imgData);

				return bitmap;
			}
		}


		[DllImport(DLL_FILE_NAME)] public static extern void Init([MarshalAs(UnmanagedType.LPStr)] string pubAddress, [MarshalAs(UnmanagedType.LPStr)] string subAddress);
		[DllImport(DLL_FILE_NAME)] public static extern void Release();

		[DllImport(DLL_FILE_NAME)] public static extern ImageData GetOriginImage();
		[DllImport(DLL_FILE_NAME)] public static extern ImageData GetStateImage();
		[DllImport(DLL_FILE_NAME)] public static extern ImageData GetFilterImage();

		[DllImport(DLL_FILE_NAME)] public static extern void TurnOff();
		[DllImport(DLL_FILE_NAME)] public static extern void StopMove();
		[DllImport(DLL_FILE_NAME)] public static extern void ChangeRearValue(int diff);
		[DllImport(DLL_FILE_NAME)] public static extern void ChangeSteerValue(float diff);
		[DllImport(DLL_FILE_NAME)] public static extern void ChangeCameraPitchValue(float diff);
		[DllImport(DLL_FILE_NAME)] public static extern void ChangeCameraPitchYaw(float diff);
	}
}

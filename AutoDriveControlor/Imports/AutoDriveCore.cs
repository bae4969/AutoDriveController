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
	internal static class AutoDriveCore
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

			public ImageData()
			{
				W = 0;
				H = 0;
				Ch = 0;
				Step = 0;
				Data = IntPtr.Zero;
			}
			public unsafe ImageData(Bitmap bitmap)
			{
				W = bitmap.Width;
				H = bitmap.Height;
				switch (bitmap.PixelFormat)
				{
					case PixelFormat.Format8bppIndexed:
						Ch = 1;
						Step = 1;
						break;
					case PixelFormat.Format16bppGrayScale:
						Ch = 1;
						Step = 2;
						break;
					case PixelFormat.Format24bppRgb:
						Ch = 3;
						Step = 1;
						break;
					case PixelFormat.Format32bppArgb:
						Ch = 4;
						Step = 1;
						break;
					default: throw new Exception();
				}

				int totMemSize = W * H * Ch * Step;
				Data = Marshal.AllocHGlobal(totMemSize);

				Rectangle rect = new(0, 0, W, H);
				BitmapData imgData = bitmap.LockBits(
					rect,
					ImageLockMode.ReadOnly,
					bitmap.PixelFormat
				);
				Buffer.MemoryCopy((void*)imgData.Scan0, (void*)Data, totMemSize, totMemSize);
				bitmap.UnlockBits(imgData);
			}
			public unsafe Bitmap ToBitmap()
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
					throw new Exception();

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

		[DllImport(DLL_FILE_NAME)]
		public static extern ImageData ApplyImageFilter(ImageData img);
	}
}

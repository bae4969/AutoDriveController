using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoDriveControlor.Classes
{
	public static class CONNECTION_STRINGS
	{
		//public const string EXTERN_PUB = "tcp://192.168.0.150:45001";
		//public const string EXTERN_SUB = "tcp://192.168.0.150:45000";

		public const string EXTERN_PUB = "tcp://BaeIptimeDDNSAddress.iptime.org:45001";
		public const string EXTERN_SUB = "tcp://BaeIptimeDDNSAddress.iptime.org:45000";
	}

	public static class PB_CAL
	{
		public static RectangleF CalZoomImagePictureBoxRectangle(Rectangle pbRect, Size imgSize)
		{
			float pbRate = (float)pbRect.Width / pbRect.Height;
			float imgRate = (float)imgSize.Width / imgSize.Height;

			RectangleF zommImgRect = pbRect;
			float zoomRate;
			if (pbRate > imgRate)
			{
				zoomRate = (float)pbRect.Height / imgSize.Height;
				zommImgRect.Width = imgSize.Width * zoomRate;
				zommImgRect.Height = pbRect.Height;
				zommImgRect.X = pbRect.X + (pbRect.Width - zommImgRect.Width) * 0.5f;
				zommImgRect.Y = pbRect.Y;
			}
			else
			{
				zoomRate = (float)pbRect.Width / imgSize.Width;
				zommImgRect.Width = pbRect.Width;
				zommImgRect.Height = imgSize.Height * zoomRate;
				zommImgRect.X = pbRect.X;
				zommImgRect.Y = pbRect.Y + (pbRect.Height - zommImgRect.Height) * 0.5f;
			}

			return zommImgRect;
		}
	}

	public static class MATH
	{
		public static double DEGREE_TO_RADIAN(double deg)
		{
			return deg * 0.0174532925199432957692369076849;
		}
		public static double RADIAN_TO_DEGREE(double rad)
		{
			return rad * 57.295779513082320876798154814105;
		}
	}
}

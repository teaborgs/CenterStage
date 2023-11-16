package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.List;

public class Constants
{
	public static final int TOLERANCE = 10;

	public static class Camera
	{
		// UNITS ARE PIXELS
		// NOTE: This calibration is for the C270 webcam at 1280x720.
		//       Base calibration was done at 640x480, so the values are doubled for 1280x720.
		public static final float CAMERA_FX = 822.317f * 2;
		public static final float CAMERA_FY = 822.317f * 1.5f;
		public static final float CAMERA_CX = 319.495f * 2;
		public static final float CAMERA_CY = 242.502f * 1.5f;

		public static final int CAMERA_WIDTH = 1280; // 640 * 2;
		public static final int CAMERA_HEIGHT = 720; // 480 * 1.5;

		// UNITS ARE METERS
		public static final double CAMERA_TAG_SIZE = 0.166;

		public static final short CAMERA_TAG_LEFT = 1;
		public static final short CAMERA_TAG_MIDDLE = 2;
		public static final short CAMERA_TAG_RIGHT = 3;
	}

	public static class Autonomous
	{
		public static class YellowDropPos
		{
			public static final Pose2d Left = new Pose2d(3, 3, 0); // tbd
			public static final Pose2d Center = new Pose2d(0, 0, 0); // tbd
			public static final Pose2d Right = new Pose2d(0, 0, 0); // tbd
		}

		public static class PurpleDropPos
		{
			public static final Pose2d Left = new Pose2d(5, 5, 3.1415926); // tbd
			public static final Pose2d Center = new Pose2d(0, 0, 0); // tbd
			public static final Pose2d Right = new Pose2d(0, 0, 0); // tbd
		}
	}

	public static class Data
	{
		public static class Tumbler
		{
			public static final int LOAD = 0;
			public static final int IDLE = 400;
			public static final int BACKDROP = 810;
			public static final int[] STACK_POSES = {1495, 1460, 1420, 1395, 1350};
		}

		public static class Lift
		{
			public static final int PICKUP = -10;
			public static final int LEVEL_1 = 950;
			public static final int LEVEL_2 = 2150;
		}

		public static class Suspender
		{
			public static final int IDLE = 0;
			public static final int SUSPEND = 1870;
			public static final int LOCK = -50;
		}

		public static class Claw
		{
			public static final double IDLE = 0.45;
			public static final double BUSY = 1;
		}

		public static class Rotator
		{
			public static final double IDLE = 0;
			public static final double BUSY = 1;
		}

		public static class Locker
		{
			public static final double IDLE = 0.5;
			public static final double BUSY = 0;
		}

		public static class Plane
		{
			public static class Leveler
			{
				public static final double IDLE = 0.5;
				public static final double BUSY = 0.7;
			}

			public static class Releaser
			{
				public static final double IDLE = 0.5;
				public static final double BUSY = 0;
			}
		}
	}

	public static class Detection
	{
		private static final Scalar WHITE_LOWER = new Scalar(0, 0, 220);
		private static final Scalar WHITE_UPPER = new Scalar(150, 110, 255);

		private static final Scalar GREEN_LOWER = new Scalar(45, 60, 100);
		private static final Scalar GREEN_UPPER = new Scalar(90, 255, 190);

		private static final Scalar PURPLE_LOWER = new Scalar(100, 85, 90);
		private static final Scalar PURPLE_UPPER = new Scalar(160, 120, 255);

		private static final Scalar YELLOW_LOWER = new Scalar(10, 100, 150);
		private static final Scalar YELLOW_UPPER = new Scalar(40, 255, 255);

		public static final Scalar RED_LOWER = new Scalar(0, 20, 30);
		public static final Scalar RED_UPPER = new Scalar(20, 255, 255);

		public static final List<Scalar> LOWER_BOUNDS = Arrays.asList(WHITE_LOWER, GREEN_LOWER, PURPLE_LOWER, YELLOW_LOWER);
		public static final List<Scalar> UPPER_BOUNDS = Arrays.asList(WHITE_UPPER, GREEN_UPPER, PURPLE_UPPER, YELLOW_UPPER);

		public static final double MIN_AREA = 0;

		public static final Scalar BLUE = new Scalar(7, 197, 235, 255);
		public static final Scalar RED = new Scalar(255, 0, 0, 255);
		public static final Scalar GREEN = new Scalar(0, 255, 0, 255);
		public static final Scalar WHITE = new Scalar(255, 255, 255, 255);
	}
}
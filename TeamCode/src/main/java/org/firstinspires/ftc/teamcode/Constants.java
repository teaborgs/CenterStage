package org.firstinspires.ftc.teamcode;

import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class Constants
{
	public static Constants instance = new Constants();
	private final HashMap<String, Object> robotConstants = new HashMap<>();

	public static void Init(Utilities.RobotType robotType)
	{
		switch (robotType) {
			case ROBOT_1:
				instance.robotConstants.clear();
				instance.robotConstants.put("tumblerLoad", 0);
				instance.robotConstants.put("tumblerIdle", 400);
				instance.robotConstants.put("tumblerBackdrop", 810);
				instance.robotConstants.put("tumblerStackPoses", new int[]{1495, 1460, 1420, 1395, 1350});
				instance.robotConstants.put("tumblerSpikeMark", 1300);
				instance.robotConstants.put("liftPickup", -10);
				instance.robotConstants.put("liftLevel1", 900);
				instance.robotConstants.put("liftLevel2", 1400);
				instance.robotConstants.put("liftLevel3", 1700);
				instance.robotConstants.put("liftLevel4", 2200);
				instance.robotConstants.put("suspenderIdle", 0);
				instance.robotConstants.put("suspenderSuspend", 1870);
				instance.robotConstants.put("suspenderLock", -50);
				instance.robotConstants.put("clawIdle", 0.45d);
				instance.robotConstants.put("clawBusy", 1d);
				instance.robotConstants.put("rotatorIdle", 0d);
				instance.robotConstants.put("rotatorBusy", 1d);
				instance.robotConstants.put("lockerIdle", 0.5d);
				instance.robotConstants.put("lockerBusy", 0d);
				instance.robotConstants.put("planeLevelerIdle", 0.5d);
				instance.robotConstants.put("planeLevelerBusy", 0.7d);
				instance.robotConstants.put("planeShooterIdle", 0.5d);
				instance.robotConstants.put("planeShooterBusy", 0d);
				instance.robotConstants.put("intakeMaxPower", 0.8d);
				instance.robotConstants.put("liftNormalPower", 1.0d);
				instance.robotConstants.put("liftSuspendPower", 1.0d);
				break;
			case ROBOT_2:
				instance.robotConstants.clear();
				instance.robotConstants.put("tumblerLoad", 0);
				instance.robotConstants.put("tumblerIdle", 400);
				instance.robotConstants.put("tumblerBackdrop", 810);
				instance.robotConstants.put("tumblerStackPoses", new int[]{1495, 1460, 1420, 1395, 1350});
				instance.robotConstants.put("tumblerSpikeMark", 1300);
				instance.robotConstants.put("liftPickup", 0);
				instance.robotConstants.put("liftLevel1", 400);
				instance.robotConstants.put("liftLevel2", 700);
				instance.robotConstants.put("liftLevel3", 900);
				instance.robotConstants.put("liftLevel4", 1200);
				instance.robotConstants.put("suspenderIdle", 0);
				instance.robotConstants.put("suspenderSuspend", 970);
				instance.robotConstants.put("suspenderLock", -10);
				instance.robotConstants.put("clawIdle", 0.45d);
				instance.robotConstants.put("clawBusy", 1d);
				instance.robotConstants.put("rotatorIdle", 0d);
				instance.robotConstants.put("rotatorBusy", 1d);
				instance.robotConstants.put("lockerIdle", 0d);
				instance.robotConstants.put("lockerBusy", 0.5d);
				instance.robotConstants.put("planeLevelerIdle", 0d);
				instance.robotConstants.put("planeLevelerBusy", 0.5d);
				instance.robotConstants.put("planeShooterIdle", 0.4d);
				instance.robotConstants.put("planeShooterBusy", 0.5d);
				instance.robotConstants.put("intakeMaxPower", 0.8d);
				instance.robotConstants.put("liftNormalPower", 0.8d);
				instance.robotConstants.put("liftSuspendPower", 1.0d);
				break;
		}
	}

	public static final int TOLERANCE = 10;

	public static class Camera
	{
		// UNITS ARE PIXELS
		// NOTE: This calibration is for the C270 webcam at 1280x720.
		//       Base calibration was done at 640x480, so the values are doubled for 1280x720.
		public static final float CAMERA_FX = 822.317f;//* 2;
		public static final float CAMERA_FY = 822.317f;//* 1.5f;
		public static final float CAMERA_CX = 319.495f;//* 2;
		public static final float CAMERA_CY = 242.502f;//* 1.5f;

		public static final int CAMERA_WIDTH = 640; //* 2;
		public static final int CAMERA_HEIGHT = 480; //* 1.5;
	}

	public static class Detection
	{
		public static class TeamProp
		{
			public static final double PROP_SIZE = 3000;
			public static final Scalar RED_LOWER = new Scalar(0, 100, 100);
			public static final Scalar RED_UPPER = new Scalar(10, 255, 255);
			public static final Scalar BLUE_LOWER = new Scalar(100, 100, 100);
			public static final Scalar BLUE_UPPER = new Scalar(140, 255, 255);
		}

		public static class AprilTag
		{
			public static final double TAG_SIZE = 0.166;
			public static final Scalar RED = new Scalar(255, 0, 0);
			public static final Scalar GREEN = new Scalar(0, 255, 0);
			public static final Scalar BLUE = new Scalar(0, 0, 255);
			public static final Scalar WHITE = new Scalar(255, 255, 255);
		}

		public static class Pixels
		{
			private static final Scalar WHITE_LOWER = new Scalar(0, 0, 220);
			private static final Scalar WHITE_UPPER = new Scalar(150, 110, 255);

			private static final Scalar GREEN_LOWER = new Scalar(45, 60, 100);
			private static final Scalar GREEN_UPPER = new Scalar(90, 255, 190);

			private static final Scalar PURPLE_LOWER = new Scalar(100, 85, 90);
			private static final Scalar PURPLE_UPPER = new Scalar(160, 120, 255);

			private static final Scalar YELLOW_LOWER = new Scalar(10, 100, 150);
			private static final Scalar YELLOW_UPPER = new Scalar(40, 255, 255);

			public static final List<Scalar> LOWER_BOUNDS = Arrays.asList(WHITE_LOWER, GREEN_LOWER, PURPLE_LOWER, YELLOW_LOWER);
			public static final List<Scalar> UPPER_BOUNDS = Arrays.asList(WHITE_UPPER, GREEN_UPPER, PURPLE_UPPER, YELLOW_UPPER);

			public static final double MIN_AREA = 2000;
		}
	}

	public static int getTumblerLoad()
	{
		return (int) instance.robotConstants.get("tumblerLoad");
	}

	public static int getTumblerIdle()
	{
		return (int) instance.robotConstants.get("tumblerIdle");
	}

	public static int getTumblerBackdrop()
	{
		return (int) instance.robotConstants.get("tumblerBackdrop");
	}

	public static int[] getTumblerStackPoses()
	{
		return (int[]) instance.robotConstants.get("tumblerStackPoses");
	}

	public static int getTumblerSpikeMark()
	{
		return (int) instance.robotConstants.get("tumblerSpikeMark");
	}

	public static int getLiftPickup()
	{
		return (int) instance.robotConstants.get("liftPickup");
	}

	public static int getLiftLevel1()
	{
		return (int) instance.robotConstants.get("liftLevel1");
	}

	public static int getLiftLevel2()
	{
		return (int) instance.robotConstants.get("liftLevel2");
	}

	public static int getLiftLevel3()
	{
		return (int) instance.robotConstants.get("liftLevel3");
	}

	public static int getLiftLevel4()
	{
		return (int) instance.robotConstants.get("liftLevel4");
	}

	public static int getSuspenderIdle()
	{
		return (int) instance.robotConstants.get("suspenderIdle");
	}

	public static int getSuspenderSuspend()
	{
		return (int) instance.robotConstants.get("suspenderSuspend");
	}

	public static int getSuspenderLock()
	{
		return (int) instance.robotConstants.get("suspenderLock");
	}

	public static double getClawIdle()
	{
		return (double) instance.robotConstants.get("clawIdle");
	}

	public static double getClawBusy()
	{
		return (double) instance.robotConstants.get("clawBusy");
	}

	public static double getRotatorIdle()
	{
		return (double) instance.robotConstants.get("rotatorIdle");
	}

	public static double getRotatorBusy()
	{
		return (double) instance.robotConstants.get("rotatorBusy");
	}

	public static double getLockerIdle()
	{
		return (double) instance.robotConstants.get("lockerIdle");
	}

	public static double getLockerBusy()
	{
		return (double) instance.robotConstants.get("lockerBusy");
	}

	public static double getPlaneLevelerIdle()
	{
		return (double) instance.robotConstants.get("planeLevelerIdle");
	}

	public static double getPlaneLevelerBusy()
	{
		return (double) instance.robotConstants.get("planeLevelerBusy");
	}

	public static double getPlaneShooterIdle()
	{
		return (double) instance.robotConstants.get("planeShooterIdle");
	}

	public static double getPlaneShooterBusy()
	{
		return (double) instance.robotConstants.get("planeShooterBusy");
	}

	public static double getIntakeMaxPower()
	{
		return (double) instance.robotConstants.get("intakeMaxPower");
	}

	public static double getLiftNormalPower()
	{
		return (double) instance.robotConstants.get("liftNormalPower");
	}

	public static double getLiftSuspendPower()
	{
		return (double) instance.robotConstants.get("liftSuspendPower");
	}
}
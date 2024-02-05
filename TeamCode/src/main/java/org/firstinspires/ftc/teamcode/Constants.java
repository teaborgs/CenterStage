package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;
import java.util.Arrays;
import java.util.List;

@Config
public class Constants
{
	public static class Container
	{
		public int TUMBLER_LOAD = 0;
		public int TUMBLER_IDLE = 0;
		public int TUMBLER_BACKDROP = 0;
		public int TUMBLER_SPIKE_MARK = 0;
		public int[] LIFT_LEVELS = new int[]{};
		public int SUSPENDER_IDLE = 0;
		public int SUSPENDER_SUSPEND = 0;
		public double CLAW_IDLE = 0;
		public double CLAW_BUSY = 0;
		public double ROTATOR_IDLE = 0;
		public double ROTATOR_BUSY = 0;
		public double PLANE_LEVELER_IDLE = 0;
		public double PLANE_LEVELER_BUSY = 0;
		public double PLANE_SHOOTER_IDLE = 0;
		public double PLANE_SHOOTER_BUSY = 0;
		public double INTAKE_MAX_POWER = 0;
		public double LIFT_NORMAL_POWER = 0;
		public double LIFT_SUSPEND_POWER = 0;
		public double BACKDROP_DISTANCE = 0;
		public double ANTENNA_IDLE = 0;
		public double ANTENNA_GUIDE = 0;
		public double ANTENNA_GRAB = 0;
		public double ANTENNA_INTAKE_RUN_TIME = 0;
		public double TUMBLER_MAX_CURRENT = 0;

		public Container() {
			switch (Globals.GetCurrentRobotType())
			{
				case ROBOT_1:
					TUMBLER_LOAD = 0;
					TUMBLER_IDLE = 400;
					TUMBLER_BACKDROP = 925;
					TUMBLER_SPIKE_MARK = 1300;
					LIFT_LEVELS = new int[] {-10, 50, 250, 500, 750, 1000};
					SUSPENDER_IDLE = 0;
					SUSPENDER_SUSPEND = 960;
					CLAW_IDLE = 0.45d;
					CLAW_BUSY = 1d;
					ROTATOR_IDLE = 0d;
					ROTATOR_BUSY = 1d;
					PLANE_LEVELER_IDLE = 0.5d;
					PLANE_LEVELER_BUSY = 0.75d;
					PLANE_SHOOTER_IDLE = 0.5d;
					PLANE_SHOOTER_BUSY = 0d;
					INTAKE_MAX_POWER = 0.8d;
					LIFT_NORMAL_POWER = 1.0d;
					LIFT_SUSPEND_POWER = 1.0d;
					BACKDROP_DISTANCE = 12.0d;
					ANTENNA_IDLE = 0.0d;
					ANTENNA_GUIDE = 0.5d;
					ANTENNA_GRAB = 0.95d;
					ANTENNA_INTAKE_RUN_TIME = 1.0d;
					TUMBLER_MAX_CURRENT = 2500.0d;
					break;
				case ROBOT_2:
					TUMBLER_LOAD = 0;
					TUMBLER_IDLE = 400;
					TUMBLER_BACKDROP = 850;
					TUMBLER_SPIKE_MARK = 1300;
					LIFT_LEVELS = new int[] {-10, 50, 250, 500, 750, 1000};
					SUSPENDER_IDLE = 0;
					SUSPENDER_SUSPEND = 1900;
					CLAW_IDLE = 0.45d;
					CLAW_BUSY = 1d;
					ROTATOR_IDLE = 0d;
					ROTATOR_BUSY = 1d;
					PLANE_LEVELER_IDLE = 0d;
					PLANE_LEVELER_BUSY = 0.5d;
					PLANE_SHOOTER_IDLE = 0.4d;
					PLANE_SHOOTER_BUSY = 0.6d;
					INTAKE_MAX_POWER = 0.8d;
					LIFT_NORMAL_POWER = 0.8d;
					LIFT_SUSPEND_POWER = 1.0d;
					BACKDROP_DISTANCE = 15.0d;
					ANTENNA_IDLE = 0.0d;
					ANTENNA_GUIDE = 0.5d;
					ANTENNA_GRAB = 0.95d;
					ANTENNA_INTAKE_RUN_TIME = 1.0d;
					TUMBLER_MAX_CURRENT = 3500.0d;
					break;
			}
		}
	}

	private static Container parameters = new Container();

	public static void Init() { parameters = new Container(); }

	public static final int TOLERANCE = 10;

	public static class Camera
	{
		/**
		 * UNITS ARE PIXELS
		 * NOTE: This calibration is for the C270 webcam at 1280x720.
		 * Base calibration was done at 640x480, so the values are doubled for 1280x720.
		 */
		public static final float CAMERA_FX = 822.317f;
		public static final float CAMERA_FY = 822.317f;
		public static final float CAMERA_CX = 319.495f;
		public static final float CAMERA_CY = 242.502f;

		public static final int CAMERA_WIDTH = 640;
		public static final int CAMERA_HEIGHT = 480;
	}

	public static class Detection
	{
		public static class TeamProp
		{
			public static final double PROP_SIZE = 6500;
			public static final Scalar RED_LOWER = new Scalar(0, 100, 100);
			public static final Scalar RED_UPPER = new Scalar(10, 255, 255);
			public static final Scalar BLUE_LOWER = new Scalar(10, 100, 100);
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

	public static int getTumblerLoad() { return parameters.TUMBLER_LOAD; }
	public static int getTumblerIdle() { return parameters.TUMBLER_IDLE; }
	public static int getTumblerBackdrop() { return parameters.TUMBLER_BACKDROP; }
	public static int getTumblerSpikeMark() { return parameters.TUMBLER_SPIKE_MARK; }
	public static int[] getLiftLevels() {	return parameters.LIFT_LEVELS;	}
	public static int getSuspenderIdle() { return parameters.SUSPENDER_IDLE; }
	public static int getSuspenderSuspend() { return parameters.SUSPENDER_SUSPEND; }
	public static double getClawIdle() { return parameters.CLAW_IDLE; }
	public static double getClawBusy() { return parameters.CLAW_BUSY; }
	public static double getRotatorIdle() { return parameters.ROTATOR_IDLE; }
	public static double getRotatorBusy() { return parameters.ROTATOR_BUSY; }
	public static double getPlaneLevelerIdle() { return parameters.PLANE_LEVELER_IDLE; }
	public static double getPlaneLevelerBusy() { return parameters.PLANE_LEVELER_BUSY; }
	public static double getPlaneShooterIdle() { return parameters.PLANE_SHOOTER_IDLE; }
	public static double getPlaneShooterBusy() { return parameters.PLANE_SHOOTER_BUSY; }
	public static double getIntakeMaxPower() { return parameters.INTAKE_MAX_POWER; }
	public static double getLiftNormalPower() {	return parameters.LIFT_NORMAL_POWER; }
	public static double getLiftSuspendPower() { return parameters.LIFT_SUSPEND_POWER; }
	public static double getBackdropDistance() { return parameters.BACKDROP_DISTANCE; }
	public static double getAntennaIdle() { return parameters.ANTENNA_IDLE; }
	public static double getAntennaGuide() { return parameters.ANTENNA_GUIDE; }
	public static double getAntennaGrab() { return parameters.ANTENNA_GRAB; }
	public static double getAntennaIntakeRunTime() { return parameters.ANTENNA_INTAKE_RUN_TIME; }
	public static double getTumblerMaxCurrent() { return parameters.TUMBLER_MAX_CURRENT; }
}
package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class Utilities
{
	public static void setTimeout(Runnable runnable, int delay)
	{
		new Thread(() -> {
			try {
				Thread.sleep(delay);
				runnable.run();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}).start();
	}

	public static double centimetersToInches(double centimeters) { return centimeters / 2.54; }

	public static double inchesToCentimeters(double inches) { return inches * 2.54; }

	/**
	 * Convert an angle to a servo position
	 * @param angle The angle to convert
	 * @param maxAngle The angle that corresponds to the maximum servo position relative to the starting position (in degrees, usually 180)
	 * @return The servo position
	 */
	public static double angleToServoPosition(double angle, double maxAngle)
	{
		return angle / maxAngle;
	}

	/**
	 * Convert an angle to a servo position (assuming the maximum angle is 180 degrees)
	 * @param angle The angle to convert
	 * @return The servo position
	 */
	public static double angleToServoPosition(double angle)
	{
		return angleToServoPosition(angle, 180);
	}


	public static Action WaitForMovementStop(RobotHardware robotHardware)
	{
		return telemetryPacket -> {
			while (!robotHardware.mecanumDrive.updatePoseEstimate().linearVel.equals(new Vector2d(0, 0)));
			return false;
		};
	}

	public static Action ApproachWithDistSensor(RobotHardware robotHardware, double distance)
	{
		return telemetryPacket -> {
			while (true)
			{
				double offset = robotHardware.distanceSensor.getDistance(DistanceUnit.CM) - distance;
				if (offset < 2)
				{
					robotHardware.mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
					return false;
				}
				robotHardware.mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(Utilities.Clamp(offset, -10, 10) / 30, 0),0));
			}
		};
	}

	public static double Clamp(double value, double lower, double higher)
	{
		return value <= lower ? lower : Math.min(value, higher);
	}

	public static Action WaitFor(double delay) { return new SleepAction(delay); }

	public static Action RunInParallel(Action... actions) { return new ParallelAction(actions); }

	public static Action RunSequentially(Action... actions) { return new SequentialAction(actions); }

	public static Scalar hsvToRgb(double h, double s, double v)
	{
		// Normalize to 0..1
		double hPrime = h / 179.0 * 360.0;
		double sPrime = s / 255.0;
		double vPrime = v / 255.0;

		double c = vPrime * sPrime;
		double hPrimeDivided = hPrime / 60.0;
		double x = c * (1 - Math.abs(hPrimeDivided % 2 - 1));

		double rPrime = 0;
		double gPrime = 0;
		double bPrime = 0;

		if (0 <= hPrimeDivided && hPrimeDivided < 1) {
			rPrime = c;
			gPrime = x;
		} else if (1 <= hPrimeDivided && hPrimeDivided < 2) {
			rPrime = x;
			gPrime = c;
		} else if (2 <= hPrimeDivided && hPrimeDivided < 3) {
			gPrime = c;
			bPrime = x;
		} else if (3 <= hPrimeDivided && hPrimeDivided < 4) {
			gPrime = x;
			bPrime = c;
		} else if (4 <= hPrimeDivided && hPrimeDivided < 5) {
			rPrime = x;
			bPrime = c;
		} else if (5 <= hPrimeDivided && hPrimeDivided < 6) {
			rPrime = c;
			bPrime = x;
		}

		double m = vPrime - c;

		int r = (int) ((rPrime + m) * 255.0);
		int g = (int) ((gPrime + m) * 255.0);
		int b = (int) ((bPrime + m) * 255.0);

		return new Scalar(r, g, b);
	}

	public static void CutPower(DcMotorEx... devices)
	{
		for (DcMotorEx device : devices) {
			device.setPower(0);
			device.setMotorDisable();
		}
	}

	public static void RestorePower(DcMotorEx... devices)
	{
		for (DcMotorEx device : devices) device.setMotorEnable();
	}

	public static void CutPower(Servo... devices)
	{
		for (Servo device : devices) device.getController().pwmDisable();
	}

	public static void RestorePower(Servo... devices)
	{
		for (Servo device : devices) device.getController().pwmEnable();
	}

	public static File MakeVideoFile(String subFolder)
	{
		SimpleDateFormat ddMMyyyy = new SimpleDateFormat("dd-MM-yyyy", Locale.getDefault());
		SimpleDateFormat hhmmss = new SimpleDateFormat("HH-mm-ss", Locale.getDefault());

		File recordings = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/recordings");
		if (!recordings.exists()) recordings.mkdir();
		File subfolder = new File(recordings.getAbsolutePath() + "/" + subFolder);
		if (!subfolder.exists()) subfolder.mkdir();
		File dateFolder = new File(subfolder.getAbsolutePath() + "/" + ddMMyyyy.format(new Date()));
		if (!dateFolder.exists()) dateFolder.mkdir();
		return new File(dateFolder.getAbsolutePath() + "/" + hhmmss.format(new Date()) + ".mp4");
	}

	public enum State
	{
		IDLE,
		BUSY
	}

	public enum DelayDirection
	{
		BEFORE,
		AFTER,
		BOTH
	}

	public enum DetectionCase
	{
		LEFT, CENTER, RIGHT
	}

	public enum Alliance
	{
		BLUE, RED
	}

	public enum RobotType
	{
		ROBOT_1,
		ROBOT_2
	}

	public enum PathType
	{
		SHORT, LONG
	}

	public enum ParkingPosition
	{
		CENTER, WALL
	}
}
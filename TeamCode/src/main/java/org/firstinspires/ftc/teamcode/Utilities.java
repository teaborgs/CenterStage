package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;

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

	public static double centimetersToInches(double centimeters)
	{
		return centimeters / 2.54;
	}

	private static RobotType robotType = null;

	public static RobotType GetCurrentRobotType(HardwareMap hardwareMap, Telemetry telemetry, Gamepad... gamepads)
	{
		if (robotType != null) return robotType;
		if (hardwareMap.get("robot1") != null) robotType = RobotType.ROBOT_1;
		else if (hardwareMap.get("robot2") != null) robotType = RobotType.ROBOT_2;
		else {
			if (telemetry == null)
				throw new Error("This mode requires the robot to be specified in the config!");
			telemetry.addLine("Please select a robot");
			telemetry.addLine("Press A for Robot 1");
			telemetry.addLine("Press B for Robot 2");
			telemetry.update();
			robotType = WaitForRobotSelection(gamepads);
		}
		return robotType;
	}

	public static Utilities.RobotType WaitForRobotSelection(Gamepad... gamepads)
	{
		Utilities.RobotType robotType = null;
		while (robotType == null) {
			for (Gamepad gamepad : gamepads) {
				if (gamepad.a) robotType = Utilities.RobotType.ROBOT_1;
				else if (gamepad.b) robotType = Utilities.RobotType.ROBOT_2;
			}
		}
		return robotType;
	}

	public static boolean IsDebugging(HardwareMap hardwareMap)
	{
		try {
			return hardwareMap.get("debug") != null;
		} catch (Exception e) {
			return false;
		}
	}

	public static Action WaitFor(double delay)
	{
		return new SleepAction(delay);
	}

	public static Action RunInParallel(Action... actions)
	{
		return new ParallelAction(actions);
	}

	public static Action RunSequentially(Action... actions)
	{
		return new SequentialAction(actions);
	}

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
		for (DcMotorEx device : devices) device.setMotorDisable();
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

	public enum State
	{
		IDLE,
		BUSY
	}

	public enum PickupMode
	{
		INTAKE,
		STACK
	}

	public enum DelayDirection
	{
		BEFORE,
		AFTER,
		BOTH
	}

	public enum DetectionCase
	{
		NONE, LEFT, CENTER, RIGHT
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
}
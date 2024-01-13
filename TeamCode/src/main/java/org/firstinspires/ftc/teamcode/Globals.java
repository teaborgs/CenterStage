package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Globals
{
	private static Utilities.RobotType robotType = null;
	private static boolean isDebugging = false;

	public static void ValidateConfig(HardwareMap hardwareMap, Telemetry telemetry, Gamepad... gamepads)
	{
		try {
			Globals.isDebugging = hardwareMap.get("debug") != null;
		} catch (Exception e) {
			Globals.isDebugging = false;
		}

		if (Globals.robotType != null) return;

		try {
			if (hardwareMap.get("robot1") != null) Globals.robotType = Utilities.RobotType.ROBOT_1;
		} catch (Exception e) {
			try {
				if (hardwareMap.get("robot2") != null) Globals.robotType = Utilities.RobotType.ROBOT_2;
			} catch (Exception e2) {
				Globals.robotType = null;
			}
		}

		if (Globals.robotType == null) {
			if (telemetry == null)
				throw new Error("This mode requires the robot to be specified in the config!");
			telemetry.addLine("Please select a robot");
			telemetry.addLine("Press A for Robot 1");
			telemetry.addLine("Press B for Robot 2");
			telemetry.update();
			Globals.robotType = WaitForRobotSelection(gamepads);
		}
	}

	public static Utilities.RobotType GetCurrentRobotType()
	{
		if (Globals.robotType == null) throw new Error("Robot type was not yet fetched!");
		return Globals.robotType;
	}

	public static boolean IsDebugging()
	{
		return Globals.isDebugging;
	}

	private static Utilities.RobotType WaitForRobotSelection(Gamepad... gamepads)
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
}

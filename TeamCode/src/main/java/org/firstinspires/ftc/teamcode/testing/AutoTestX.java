package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/*
 * Move 1 meter on the X axis
 */
@Autonomous(name = "Auto Test X", group = "Testing")
public class AutoTestX extends LinearOpMode
{
	private MecanumDrive mecanumDrive;

	@Override
	public void runOpMode()
	{
		Init();
		waitForStart();
		Run();
	}

	private void Init()
	{
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2);
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), Globals.GetCurrentRobotType());
	}

	private void Run()
	{
		Actions.runBlocking(mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
				.lineToX(39.37 * 2)
				.build());
	}
}

package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities;

@Autonomous(name = "Auto Test Turn", group = "Testing")
public class AutoTestTurn extends LinearOpMode
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
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), Utilities.GetCurrentRobotType(hardwareMap, telemetry, gamepad1, gamepad2));
	}

	private void Run()
	{
		Actions.runBlocking(mecanumDrive.actionBuilder(new Pose2d(0,0, 0))
				.turn(Math.PI / 2)
				.build());
	}
}

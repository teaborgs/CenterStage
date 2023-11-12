package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Auto Test X", group = "Testing")
public class AutoTestX extends LinearOpMode
{
	MecanumDrive mecanumDrive;

	@Override
	public void runOpMode()
	{
		Init();
		waitForStart();
		Run();
	}

	private void Init()
	{
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
	}

	private void Run()
	{
		Actions.runBlocking(mecanumDrive.actionBuilder(new Pose2d(0,0, 0))
				.lineToX(39.37 * 2)
				.build());
		mecanumDrive.updatePoseEstimate();
		telemetry.addData("x", mecanumDrive.pose.position.x);
		telemetry.addData("y", mecanumDrive.pose.position.y);
		telemetry.addData("heading", mecanumDrive.pose.heading);
		telemetry.update();
		Actions.runBlocking(new SleepAction(10));
	}
}

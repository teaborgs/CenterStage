package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class AutoTest extends LinearOpMode
{
	MecanumDrive mecanumDrive;

	@Override
	public void runOpMode()
	{
		Init();
		Run();
	}

	private void Init()
	{
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
	}

	private void Run()
	{
		Actions.runBlocking(mecanumDrive.actionBuilder(new Pose2d(0,0, 0))
				.lineToX(10)
				.build());
	}
}

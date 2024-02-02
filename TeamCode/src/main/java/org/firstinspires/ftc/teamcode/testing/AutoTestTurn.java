package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities;

@Autonomous(name = "Auto Test Turn", group = "Testing")
public class AutoTestTurn extends BaseOpMode
{
	private MecanumDrive mecanumDrive;

	@Override
	protected void OnInitialize()
	{
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2);
		mecanumDrive = new MecanumDrive(hardwareMap);
	}

	@Override
	protected void OnRun()
	{
		Actions.runBlocking(
				mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
						.turn(Math.PI * 10) // 5 full rotations
						.build()
		);
	}
}

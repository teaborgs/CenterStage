package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/*
 * Move 2 meters on the Y axis
 */
@Autonomous(name = "Auto Test Y", group = "Testing")
public class AutoTestY extends BaseOpMode
{
	private MecanumDrive mecanumDrive;

	@Override
	protected void OnInitialize()
	{
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2);
		setAutonomous(true);
		mecanumDrive = new MecanumDrive(hardwareMap);
	}

	@Override
	protected void OnRun()
	{
		Actions.runBlocking(mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
				.setTangent(Math.PI / 2)
				.lineToY(39.37 * 2)
				.build());
	}
}

package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


@Autonomous(name = "Auto Test Spline", group = "Testing")
public class AutoTestSpline extends BaseOpMode
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
		Actions.runBlocking(
				mecanumDrive.actionBuilder(mecanumDrive.pose)
						.splineTo(new Vector2d(30, 30), Math.PI / 2)
						.splineTo(new Vector2d(60, 0), Math.PI)
						.build());
	}
}

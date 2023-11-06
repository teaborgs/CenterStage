package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities;

@Autonomous(name = "LowAuto", group = "Auto")
public class LowAuto extends LinearOpMode
{
	MecanumDrive mecanumDrive;
	DcMotorEx intakeMotor;
	DcMotorEx liftMotor1, liftMotor2;
	DcMotorEx tumblerMotor;
	Servo clawServo, rotatorServo;

	Utilities.DetectionCase detectedCase = Utilities.DetectionCase.NONE;

	Action purpleDrop, yellowDrop;

	@Override
	public void runOpMode()
	{
		Init();

		while (!isStarted())
			Detect();

		SelectPositions();

		Run();
	}

	private void Init()
	{
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

		intakeMotor = hardwareMap.get(DcMotorEx.class, "slot4");
		intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

		liftMotor1 = hardwareMap.get(DcMotorEx.class, "slot5");
		liftMotor2 = hardwareMap.get(DcMotorEx.class, "slot6");
		liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		liftMotor1.setDirection(DcMotorEx.Direction.REVERSE);
		liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		liftMotor1.setTargetPosition(Constants.Data.Suspender.IDLE);
		liftMotor2.setTargetPosition(Constants.Data.Suspender.IDLE);
		liftMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

		tumblerMotor = hardwareMap.get(DcMotorEx.class, "slot7");
		tumblerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		tumblerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		tumblerMotor.setDirection(DcMotorEx.Direction.REVERSE);
		tumblerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		tumblerMotor.setTargetPosition(Constants.Data.Tumbler.IDLE);
		tumblerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

		rotatorServo = hardwareMap.get(Servo.class, "servo0");
		clawServo = hardwareMap.get(Servo.class, "servo1");
	}

	private void Detect()
	{
		// detection

		detectedCase = Utilities.DetectionCase.LEFT;
	}

	private void SelectPositions()
	{
		if (detectedCase == Utilities.DetectionCase.NONE)
		{
			detectedCase = Utilities.DetectionCase.CENTER; // default case

			telemetry.addLine("Detection Failure!");
			telemetry.update();
		}

		switch (detectedCase)
		{
			case LEFT:
				purpleDrop = mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
						.lineToXConstantHeading(Constants.Autonomous.PurpleDropPos.Left.component1().x)
						.lineToYConstantHeading(Constants.Autonomous.PurpleDropPos.Left.component1().y)
						.turnTo(Constants.Autonomous.PurpleDropPos.Left.component2())
						.build();

				yellowDrop = mecanumDrive.actionBuilder(Constants.Autonomous.PurpleDropPos.Left)
						.lineToXConstantHeading(Constants.Autonomous.YellowDropPos.Left.component1().x)
						.lineToYConstantHeading(Constants.Autonomous.YellowDropPos.Left.component1().y)
						.turnTo(Constants.Autonomous.YellowDropPos.Left.component2())
						.build();
				break;

			case CENTER:

				break;

			case RIGHT:

				break;
		}
	}

	void Run()
	{
		Action action = mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
				.lineToXConstantHeading(10)
				.build();
		Actions.runBlocking(action);
	}
}
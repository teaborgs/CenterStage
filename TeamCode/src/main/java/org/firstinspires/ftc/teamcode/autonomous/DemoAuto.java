package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;
import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;
import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Demo Auto", group = "Auto")
public class DemoAuto extends LinearOpMode
{
	// Wheel motors
	private MecanumDrive mecanumDrive;

	// Intake Motor
	private DcMotorEx intakeMotor;

	// Lift Motors
	private DcMotorEx liftMotor1, liftMotor2;

	// Tumbler Motor
	private DcMotorEx tumblerMotor;

	// Servos
	private Servo clawServo, rotatorServo, lockerServo, planeLevelServo, planeReleaseServo;

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
		lockerServo = hardwareMap.get(Servo.class, "servo2");
		planeReleaseServo = hardwareMap.get(Servo.class, "servo3");
		planeLevelServo = hardwareMap.get(Servo.class, "servo4");

		planeReleaseServo.setPosition(Constants.Data.Plane.Releaser.IDLE);
		lockerServo.setPosition(Constants.Data.Locker.IDLE);
		clawServo.setPosition(Constants.Data.Claw.IDLE);
		rotatorServo.setPosition(Constants.Data.Rotator.IDLE);

		telemetry.setMsTransmissionInterval(50);
	}

	private void Run()
	{
		Actions.runBlocking(new SequentialAction(
				telemetryPacket -> {
					clawServo.setPosition(Constants.Data.Claw.BUSY);
					return false;
				},
				new SleepAction(0.5),
				new ParallelAction(
						mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
								.lineToX(-centimetersToInches(60))
								.build(),
						new SequentialAction(
								new SleepAction(0.1),
								telemetryPacket -> {
									tumblerMotor.setPower(1);
									tumblerMotor.setTargetPosition(1300);
									return false;
								},
								new ParallelAction(
										telemetryPacket -> Math.abs(tumblerMotor.getCurrentPosition() - tumblerMotor.getTargetPosition()) > TOLERANCE,
										new SequentialAction(
												new SleepAction(0.1),
												telemetryPacket -> {
													rotatorServo.setPosition(Constants.Data.Rotator.BUSY);
													return false;
												}
										)
								),
								telemetryPacket -> {
									tumblerMotor.setPower(0.05);
									return false;
								}
						)
				),
				telemetryPacket -> {
					clawServo.setPosition(Constants.Data.Claw.IDLE);
					return false;
				},
				new SleepAction(0.1),
				telemetryPacket -> {
					rotatorServo.setPosition(Constants.Data.Rotator.IDLE);
					tumblerMotor.setPower(1);
					tumblerMotor.setTargetPosition(Constants.Data.Tumbler.LOAD);
					return false;
				},
				telemetryPacket -> Math.abs(tumblerMotor.getCurrentPosition() - tumblerMotor.getTargetPosition()) > TOLERANCE,
				telemetryPacket -> {
					tumblerMotor.setPower(0.05);
					return false;
				}
		));
	}
}

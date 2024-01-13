package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware
{
	// Wheel motors
	public final MecanumDrive mecanumDrive;

	// Intake Motor
	public final DcMotorEx intakeMotor;

	// Lift Motors
	public final DcMotorEx liftMotor1, liftMotor2;

	// Tumbler Motor
	public final DcMotorEx tumblerMotor;

	// Servos
	public final Servo clawServo, rotatorServo, planeLevelServo, planeShooterServo, antennaServo;

	// Distance Sensor
	public final Rev2mDistanceSensor distanceSensor;

	public RobotHardware(HardwareMap hardwareMap)
	{
		// Wheel motors
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0), Globals.GetCurrentRobotType());

		// Intake Motor
		intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
		intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

		// Lift Motors
		liftMotor1 = hardwareMap.get(DcMotorEx.class, "lift1");
		liftMotor2 = hardwareMap.get(DcMotorEx.class, "lift2");
		liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		liftMotor2.setDirection(DcMotorEx.Direction.REVERSE);
		liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		liftMotor1.setTargetPosition(Constants.getSuspenderIdle());
		liftMotor2.setTargetPosition(Constants.getSuspenderIdle());
		liftMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

		// Tumbler Motor
		tumblerMotor = hardwareMap.get(DcMotorEx.class, "tumbler");
		tumblerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		tumblerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		tumblerMotor.setDirection(DcMotorEx.Direction.REVERSE);
		tumblerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		tumblerMotor.setTargetPosition(Constants.getTumblerIdle());
		tumblerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

		// Servos
		clawServo = hardwareMap.get(Servo.class, "claw");
		rotatorServo = hardwareMap.get(Servo.class, "rotator");
		planeLevelServo = hardwareMap.get(Servo.class, "leveler");
		planeShooterServo = hardwareMap.get(Servo.class, "shooter");
		antennaServo = hardwareMap.get(Servo.class, "antenna");

		planeShooterServo.setPosition(Constants.getPlaneShooterIdle());
		clawServo.setPosition(Constants.getClawIdle());
		rotatorServo.setPosition(Constants.getRotatorIdle());
		antennaServo.setPosition(Constants.getAntennaIdle());
		planeLevelServo.setPosition(Constants.getPlaneShooterIdle());

		// Distance Sensor
		distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
	}
}

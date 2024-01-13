package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/*
 * Test the encoder value of a motor
 */
@TeleOp(name = "Value Test", group = "Testing")
public class ValueTest extends BaseOpMode
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
	private Servo clawServo, rotatorServo, lockerServo, planeLevelServo, planeShooterServo;

	@Override
	protected void OnInitialize()
	{
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2);
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), Globals.GetCurrentRobotType());
		Constants.Init(Globals.GetCurrentRobotType());

		intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
		intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

		liftMotor1 = hardwareMap.get(DcMotorEx.class, "lift1");
		liftMotor2 = hardwareMap.get(DcMotorEx.class, "lift2");
		liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		liftMotor1.setDirection(DcMotorEx.Direction.REVERSE);

		tumblerMotor = hardwareMap.get(DcMotorEx.class, "tumbler");
		tumblerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		tumblerMotor.setDirection(DcMotorEx.Direction.REVERSE);
		tumblerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		rotatorServo = hardwareMap.get(Servo.class, "rotator");
		clawServo = hardwareMap.get(Servo.class, "claw");
		lockerServo = hardwareMap.get(Servo.class, "locker");
		planeShooterServo = hardwareMap.get(Servo.class, "shooter");
		planeLevelServo = hardwareMap.get(Servo.class, "leveler");
	}

	@Override
	protected void OnRun()
	{
		telemetry.addData("[DEBUG] Lift 1", liftMotor1.getCurrentPosition());
		telemetry.addData("[DEBUG] Lift 2", liftMotor2.getCurrentPosition());
		telemetry.addData("[DEBUG] Tumbler", tumblerMotor.getCurrentPosition());
		telemetry.addData("[DEBUG] Rotator", rotatorServo.getPosition());
		telemetry.addData("[DEBUG] Claw", clawServo.getPosition());
		telemetry.addData("[DEBUG] Locker", lockerServo.getPosition());
		telemetry.addData("[DEBUG] Plane Level", planeLevelServo.getPosition());
		telemetry.addData("[DEBUG] Plane Release", planeShooterServo.getPosition());
		telemetry.update();
	}
}

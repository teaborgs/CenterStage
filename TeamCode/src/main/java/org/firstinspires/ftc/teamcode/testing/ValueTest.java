package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.Utilities.GetCurrentRobotType;
import static org.firstinspires.ftc.teamcode.Utilities.IsDebugging;

import com.acmerobotics.roadrunner.Pose2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities;

/*
 * Test the encoder value of a motor
 */

@Photon
@TeleOp(name = "Value Test", group = "Testing")
public class ValueTest extends BaseOpMode
{
	// Wheel motors
	private MecanumDrive mecanumDrive;

	// Intake Motor
	private PhotonDcMotor intakeMotor;

	// Lift Motors
	private PhotonDcMotor liftMotor1, liftMotor2;

	// Tumbler Motor
	private PhotonDcMotor tumblerMotor;

	// Servos
	private PhotonServo clawServo, rotatorServo, lockerServo, planeLevelServo, planeShooterServo;

	@Override
	protected void OnInitialize()
	{
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), GetCurrentRobotType(hardwareMap, telemetry, gamepad1, gamepad2));
		Constants.Init(GetCurrentRobotType(hardwareMap, telemetry, gamepad1, gamepad2));

		intakeMotor = hardwareMap.get(PhotonDcMotor.class, "intake");
		intakeMotor.setMode(PhotonDcMotor.RunMode.RUN_WITHOUT_ENCODER);

		liftMotor1 = hardwareMap.get(PhotonDcMotor.class, "lift1");
		liftMotor2 = hardwareMap.get(PhotonDcMotor.class, "lift2");
		liftMotor1.setMode(PhotonDcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor2.setMode(PhotonDcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor1.setMode(PhotonDcMotor.RunMode.RUN_USING_ENCODER);
		liftMotor2.setMode(PhotonDcMotor.RunMode.RUN_USING_ENCODER);
		liftMotor1.setDirection(PhotonDcMotor.Direction.REVERSE);

		tumblerMotor = hardwareMap.get(PhotonDcMotor.class, "tumbler");
		tumblerMotor.setMode(PhotonDcMotor.RunMode.STOP_AND_RESET_ENCODER);
		if (GetCurrentRobotType(hardwareMap, telemetry, gamepad1, gamepad2) == Utilities.RobotType.ROBOT_1)
			tumblerMotor.setDirection(PhotonDcMotor.Direction.REVERSE);
		tumblerMotor.setMode(PhotonDcMotor.RunMode.RUN_USING_ENCODER);

		rotatorServo = hardwareMap.get(PhotonServo.class, "rotator");
		clawServo = hardwareMap.get(PhotonServo.class, "claw");
		lockerServo = hardwareMap.get(PhotonServo.class, "locker");
		planeShooterServo = hardwareMap.get(PhotonServo.class, "shooter");
		planeLevelServo = hardwareMap.get(PhotonServo.class, "leveler");
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

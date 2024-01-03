package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.Utilities.GetCurrentRobotType;
import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities;

import java.util.LinkedList;

@Photon
@TeleOp(name = "Manual Drive Test", group = "Testing")
public final class ManualDriveTest extends BaseOpMode
{
	/**
	 * === Configuration ===
	 * Control Hub:
	 * > Servos:
	 * slot0 - locker (locker)
	 * slot1 - plane release (shooter)
	 * slot2 - plane level (leveler)
	 * > Motors:
	 * slot0 - left-front wheel (leftFront)
	 * slot1 - left-back wheel (leftBack)
	 * slot2 - right-back wheel (rightBack)
	 * slot3 - right-front wheel (rightFront)
	 * slot0 - left odometry
	 * slot1 - right odometry
	 * slot2 - back odometry
	 * Expansion Hub:
	 * > Servos:
	 * slot0 - rotator (rotator)
	 * slot1 - claw (claw)
	 * > Motors:
	 * slot0 - intake (intake)
	 * slot1 - lift1 (lift1)
	 * slot2 - lift2 (lift2)
	 * slot3 - tumbler (tumbler)
	 */

	// Wheel motors
	private MecanumDrive mecanumDrive;

	// Intake Motor
	private PhotonDcMotor intakeMotor;

	// Lift Motors
	private final LinkedList<PhotonDcMotor> liftMotors = new LinkedList<>();

	// Tumbler Motor
	private PhotonDcMotor tumblerMotor;

	// Servos
	private PhotonServo clawServo, rotatorServo, planeLevelServo, planeReleaseServo, antennaServo;

	// Input System
	private InputSystem wheelInput, armInput;

	private static final class Bindings
	{
		private static final class Wheel
		{
			private static final InputSystem.Key TURBO_KEY = new InputSystem.Key("right_bumper");
			private static final InputSystem.Axis DRIVE_AXIS_X = new InputSystem.Axis("left_stick_x");
			private static final InputSystem.Axis DRIVE_AXIS_Y = new InputSystem.Axis("left_stick_y");
			private static final InputSystem.Axis ROTATE_AXIS_L = new InputSystem.Axis("left_trigger");
			private static final InputSystem.Axis ROTATE_AXIS_R = new InputSystem.Axis("right_trigger");
			private static final InputSystem.Key INTAKE_KEY = new InputSystem.Key("a");
			private static final InputSystem.Key INTAKE_REVERSE_KEY = new InputSystem.Key("b");
			private static final InputSystem.Key INTAKE_CLAW = new InputSystem.Key("x");
		}

		private static final class Arm
		{
			private static final InputSystem.Key CLAW_KEY = new InputSystem.Key("a");
			private static final InputSystem.Key ROTATOR_KEY = new InputSystem.Key("b");
			private static final InputSystem.BindingCombo PLANE_COMBO = new InputSystem.BindingCombo("_plane", new InputSystem.Axis("left_trigger"), new InputSystem.Axis("right_trigger"));
			private static final InputSystem.Key TUMBLER_LOAD_KEY = new InputSystem.Key("dpad_down");
			private static final InputSystem.Key TUMBLER_BACKDROP_KEY = new InputSystem.Key("dpad_up");
			private static final InputSystem.Key TUMBLER_IDLE_KEY = new InputSystem.Key("dpad_left");
			private static final InputSystem.Key TUMBLER_STACK_KEY = new InputSystem.Key("dpad_right");
			private static final InputSystem.Axis LIFT_AXIS = new InputSystem.Axis("right_stick_y");
		}
	}

	@Override
	protected void OnInitialize()
	{
		Constants.Init(GetCurrentRobotType(hardwareMap, telemetry, gamepad1, gamepad2));
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), Utilities.GetCurrentRobotType(hardwareMap, telemetry, gamepad1, gamepad2));

		intakeMotor = hardwareMap.get(PhotonDcMotor.class, "intake");
		intakeMotor.setMode(PhotonDcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intakeMotor.setDirection(PhotonDcMotor.Direction.REVERSE);

		liftMotors.add(hardwareMap.get(PhotonDcMotor.class, "lift1"));
		liftMotors.add(hardwareMap.get(PhotonDcMotor.class, "lift2"));
		liftMotors.forEach(motor -> {
			motor.setMode(PhotonDcMotor.RunMode.STOP_AND_RESET_ENCODER);
			motor.setMode(PhotonDcMotor.RunMode.RUN_USING_ENCODER);
			motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		});
		liftMotors.get(0).setDirection(PhotonDcMotor.Direction.REVERSE);
		if(GetCurrentRobotType(hardwareMap, telemetry, gamepad1, gamepad2) == Utilities.RobotType.ROBOT_2) liftMotors.get(1).setDirection(PhotonDcMotor.Direction.REVERSE);

		tumblerMotor = hardwareMap.get(PhotonDcMotor.class, "tumbler");
		tumblerMotor.setZeroPowerBehavior(PhotonDcMotor.ZeroPowerBehavior.BRAKE);
		if(GetCurrentRobotType(hardwareMap, telemetry, gamepad1, gamepad2) == Utilities.RobotType.ROBOT_1)tumblerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		tumblerMotor.setMode(PhotonDcMotor.RunMode.STOP_AND_RESET_ENCODER);
		tumblerMotor.setMode(PhotonDcMotor.RunMode.RUN_USING_ENCODER);
		tumblerMotor.setTargetPosition(0);
		tumblerMotor.setMode(PhotonDcMotor.RunMode.RUN_TO_POSITION);

		rotatorServo = hardwareMap.get(PhotonServo.class, "rotator");
		clawServo = hardwareMap.get(PhotonServo.class, "claw");
		antennaServo = hardwareMap.get(PhotonServo.class, "antenna");

		clawServo.setPosition(Constants.getClawIdle());
		rotatorServo.setPosition(Constants.getRotatorIdle());
		antennaServo.setPosition(0);

		wheelInput = new InputSystem(gamepad1);
		armInput = new InputSystem(gamepad2);

		telemetry.setMsTransmissionInterval(50);
	}

	@Override
	protected void OnRun()
	{
		Wheels();
		Intake();
		Lift();
		Claw();
		Rotator();
		Tumbler();
		//Plane();
		Telemetry();
	}

	// ================ Wheels ================
	private void Wheels()
	{
		boolean turbo = wheelInput.isPressed(Bindings.Wheel.TURBO_KEY);

		double angle = (wheelInput.getValue(Bindings.Wheel.ROTATE_AXIS_L) - wheelInput.getValue(Bindings.Wheel.ROTATE_AXIS_R)) * (turbo ? 0.065 : 0.08);
		Vector2d wheelVel = new Vector2d(
				wheelInput.getValue(Bindings.Wheel.DRIVE_AXIS_Y),
				wheelInput.getValue(Bindings.Wheel.DRIVE_AXIS_X)
		).times(turbo ? 0.6 : 1.0);

		// Drive
		mecanumDrive.setDrivePowers(new PoseVelocity2d(wheelVel, angle));
	}

	short antennaIndex = 0;
	private void Intake()
	{
		intakeMotor.setPower(wheelInput.isPressed(Bindings.Wheel.INTAKE_KEY) ? Constants.getIntakeMaxPower() : wheelInput.isPressed(Bindings.Wheel.INTAKE_REVERSE_KEY) ? -Constants.getIntakeMaxPower() : 0);
		if(wheelInput.wasPressedThisFrame(Bindings.Wheel.INTAKE_CLAW)) {
			antennaIndex++;
			if(antennaIndex > 1) antennaIndex = 0;
		}
		switch (antennaIndex)
		{
			case 0:
				antennaServo.setPosition(0);
				break;
			case 1:
				antennaServo.setPosition(0.5);
				break;
		}
	}

	// ================ Arm ================
	private void Lift()
	{
		liftMotors.forEach(motor -> motor.setPower(armInput.getValue(Bindings.Arm.LIFT_AXIS)));
	}

	int stackTaps = 0;
	private void Tumbler()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.TUMBLER_BACKDROP_KEY))
		{
			tumblerMotor.setTargetPosition(Constants.getTumblerBackdrop());
			stackTaps = 0;
		}
		else if (armInput.wasPressedThisFrame(Bindings.Arm.TUMBLER_LOAD_KEY))
		{
			tumblerMotor.setTargetPosition(Constants.getTumblerLoad());
			stackTaps = 0;
		}
		else if (armInput.wasPressedThisFrame(Bindings.Arm.TUMBLER_IDLE_KEY))
		{
			tumblerMotor.setTargetPosition(Constants.getTumblerIdle());
			stackTaps = 0;
		}
		else if (armInput.wasPressedThisFrame(Bindings.Arm.TUMBLER_STACK_KEY))
		{
			stackTaps++;
			if (stackTaps > 5)
				stackTaps = 1;

			int targetPos = Constants.getTumblerStackPoses()[stackTaps - 1];
			tumblerMotor.setTargetPosition(targetPos);
		}

		if (Math.abs(tumblerMotor.getTargetPosition() - tumblerMotor.getCurrentPosition()) > 10)
			tumblerMotor.setPower(0.8);
		else
			tumblerMotor.setPower(0.05);
		tumblerMotor.setMode(PhotonDcMotor.RunMode.RUN_TO_POSITION);
	}

	private Utilities.State clawState = Utilities.State.BUSY;

	private void Claw()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.CLAW_KEY))
			clawState = clawState == Utilities.State.IDLE ? Utilities.State.BUSY : Utilities.State.IDLE;

		if (clawState == Utilities.State.IDLE)
			clawServo.setPosition(0);
		else
			clawServo.setPosition(0.5);
	}

	private Utilities.State rotatorState = Utilities.State.BUSY;

	private void Rotator()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.ROTATOR_KEY))
			rotatorState = rotatorState == Utilities.State.IDLE ? Utilities.State.BUSY : Utilities.State.IDLE;
		rotatorServo.setPosition(rotatorState == Utilities.State.IDLE ? 1.0 : 0.0);
	}

	private boolean planeLeveled = false;
	private boolean planeLaunched = false;
	private volatile Utilities.State planeLevelState = Utilities.State.BUSY;

	private void Plane()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.PLANE_COMBO) && !planeLeveled)
		{
			planeLevelState = Utilities.State.IDLE;
			planeLeveled = true;
		}
		planeLevelServo.setPosition(planeLevelState == Utilities.State.IDLE ? 0.5 : 0.0);
		if (planeLevelServo.getPosition() == 0.5 && !planeLaunched && planeLeveled)
		{
			planeLaunched = true;
			setTimeout(() -> {
				planeReleaseServo.setPosition(0.0);
				setTimeout(() -> {
					planeLevelState = Utilities.State.BUSY;
					planeReleaseServo.setPosition(0.5);
				}, 300);
			}, 300);
		}
	}

	private void Telemetry()
	{
		telemetry.addData("Robot pos: ", mecanumDrive.updatePoseEstimate().component1());

		telemetry.addData("Intake Power: ", intakeMotor.getPower());

		telemetry.addData("leveled", planeLeveled);
		telemetry.addData("launched", planeLaunched);

		liftMotors.forEach(motor -> telemetry.addData("Lift " + (liftMotors.indexOf(motor) + 1) + " Power: ", motor.getPower()));
		liftMotors.forEach(motor -> telemetry.addData("Lift " + (liftMotors.indexOf(motor) + 1) + " Pos: ", motor.getCurrentPosition()));

		telemetry.addData("Tumbler Power: ", tumblerMotor.getPower());
		telemetry.addData("Tumbler Pos: ", tumblerMotor.getCurrentPosition());

		telemetry.addData("Rotator Pos: ", rotatorServo.getPosition());
		telemetry.addData("Claw Pos: ", clawServo.getPosition());

		telemetry.update();
	}
}
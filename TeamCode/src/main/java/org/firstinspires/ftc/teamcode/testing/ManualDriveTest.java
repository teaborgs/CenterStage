package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities;

import java.util.LinkedList;

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

	private RobotHardware robotHardware;
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
		}

		private static final class Arm
		{
			private static final InputSystem.Key CLAW_KEY = new InputSystem.Key("a");
			private static final InputSystem.Key ROTATOR_KEY = new InputSystem.Key("b");
			private static final InputSystem.Key LOCKER_KEY = new InputSystem.Key("y");
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
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2);
		Constants.Init();
		robotHardware = new RobotHardware(hardwareMap);

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
		Plane();
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
		robotHardware.mecanumDrive.setDrivePowers(new PoseVelocity2d(wheelVel, angle));
	}

	private void Intake()
	{
		robotHardware.intakeMotor.setPower(wheelInput.isPressed(Bindings.Wheel.INTAKE_KEY) ? Constants.getIntakeMaxPower() : 0.0);
	}

	// ================ Arm ================
	private void Lift()
	{
		robotHardware.liftMotor1.setPower(armInput.getValue(Bindings.Arm.LIFT_AXIS));
		robotHardware.liftMotor2.setPower(armInput.getValue(Bindings.Arm.LIFT_AXIS));
	}

	int stackTaps = 0;
	private void Tumbler()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.TUMBLER_BACKDROP_KEY))
		{
			robotHardware.tumblerMotor.setTargetPosition(Constants.getTumblerBackdrop());
			stackTaps = 0;
		}
		else if (armInput.wasPressedThisFrame(Bindings.Arm.TUMBLER_LOAD_KEY))
		{
			robotHardware.tumblerMotor.setTargetPosition(Constants.getTumblerLoad());
			stackTaps = 0;
		}
		else if (armInput.wasPressedThisFrame(Bindings.Arm.TUMBLER_IDLE_KEY))
		{
			robotHardware.tumblerMotor.setTargetPosition(Constants.getTumblerIdle());
			stackTaps = 0;
		}
		else if (armInput.wasPressedThisFrame(Bindings.Arm.TUMBLER_STACK_KEY))
		{
			stackTaps++;
			if (stackTaps > 5)
				stackTaps = 1;

			int targetPos = Constants.getTumblerStackPoses()[stackTaps - 1];
			robotHardware.tumblerMotor.setTargetPosition(targetPos);
		}

		if (Math.abs(robotHardware.tumblerMotor.getTargetPosition() - robotHardware.tumblerMotor.getCurrentPosition()) > 10)
			robotHardware.tumblerMotor.setPower(0.8);
		else
			robotHardware.tumblerMotor.setPower(0.05);
		robotHardware.tumblerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
	}

	private Utilities.State clawState = Utilities.State.BUSY;

	private void Claw()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.CLAW_KEY))
			clawState = clawState == Utilities.State.IDLE ? Utilities.State.BUSY : Utilities.State.IDLE;
		robotHardware.clawServo.setPosition(clawState == Utilities.State.IDLE ? 0.5 : 0.0);
	}

	private Utilities.State rotatorState = Utilities.State.BUSY;

	private void Rotator()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.ROTATOR_KEY))
			rotatorState = rotatorState == Utilities.State.IDLE ? Utilities.State.BUSY : Utilities.State.IDLE;
		robotHardware.rotatorServo.setPosition(rotatorState == Utilities.State.IDLE ? 1.0 : 0.0);
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
		robotHardware.planeLevelServo.setPosition(planeLevelState == Utilities.State.IDLE ? 0.5 : 0.0);
		if (robotHardware.planeLevelServo.getPosition() == 0.5 && !planeLaunched && planeLeveled)
		{
			planeLaunched = true;
			setTimeout(() -> {
				robotHardware.planeShooterServo.setPosition(0.0);
				setTimeout(() -> {
					planeLevelState = Utilities.State.BUSY;
					robotHardware.planeLevelServo.setPosition(0.5);
				}, 300);
			}, 300);
		}
	}

	private void Telemetry()
	{
		telemetry.addData("Robot pos: ", robotHardware.mecanumDrive.updatePoseEstimate().component1());

		telemetry.addData("Intake Power: ", robotHardware.intakeMotor.getPower());

		telemetry.addData("leveled", planeLeveled);
		telemetry.addData("launched", planeLaunched);

		telemetry.addData("Lift 1 Power: ", robotHardware.liftMotor1.getPower());
		telemetry.addData("Lift 1 Pos: ", robotHardware.liftMotor1.getCurrentPosition());
		telemetry.addData("Lift 2 Power: ", robotHardware.liftMotor2.getPower());
		telemetry.addData("Lift 2 Pos: ", robotHardware.liftMotor2.getCurrentPosition());

		telemetry.addData("Tumbler Power: ", robotHardware.tumblerMotor.getPower());
		telemetry.addData("Tumbler Pos: ", robotHardware.tumblerMotor.getCurrentPosition());

		telemetry.addData("Rotator Pos: ", robotHardware.rotatorServo.getPosition());
		telemetry.addData("Claw Pos: ", robotHardware.clawServo.getPosition());
		telemetry.addData("PlaneLevel Pos: ", robotHardware.planeLevelServo.getPosition());
		telemetry.addData("PlaneRelease Pos: ", robotHardware.planeShooterServo.getPosition());

		telemetry.update();
	}
}
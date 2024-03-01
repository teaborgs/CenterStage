package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities;

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
			private static final InputSystem.BindingCombo PLANE_COMBO = new InputSystem.BindingCombo("_plane", new InputSystem.Axis("left_trigger"), new InputSystem.Axis("right_trigger"));
			private static final InputSystem.Key TUMBLER_LOAD_KEY = new InputSystem.Key("dpad_down");
			private static final InputSystem.Key TUMBLER_BACKDROP_KEY = new InputSystem.Key("dpad_up");
			private static final InputSystem.Key TUMBLER_IDLE_KEY = new InputSystem.Key("dpad_left");
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
		robotHardware.intakeSystem.SetIntakePower(wheelInput.isPressed(Bindings.Wheel.INTAKE_KEY) ? Constants.getIntakeMaxPower() : 0.0);
	}

	// ================ Arm ================
	private void Lift()
	{
		robotHardware.liftSystem.SetPower(armInput.getValue(Bindings.Arm.LIFT_AXIS));
	}

	private void Tumbler()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.TUMBLER_BACKDROP_KEY))
			robotHardware.tumblerSystem.SetPosition(Constants.getTumblerBackdrop());
		else if (armInput.wasPressedThisFrame(Bindings.Arm.TUMBLER_LOAD_KEY))
			robotHardware.tumblerSystem.SetPosition(Constants.getTumblerLoad());
		else if (armInput.wasPressedThisFrame(Bindings.Arm.TUMBLER_IDLE_KEY))
			robotHardware.tumblerSystem.SetPosition(Constants.getTumblerIdle());
	}

	private Utilities.State clawState = Utilities.State.BUSY;

	private void Claw()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.CLAW_KEY))
			clawState = clawState == Utilities.State.IDLE ? Utilities.State.BUSY : Utilities.State.IDLE;
		robotHardware.clawSystem1.SetPosition(clawState == Utilities.State.IDLE ? Constants.getClawIdle() : Constants.getClawBusy());
	}

	private Utilities.State rotatorState = Utilities.State.BUSY;

	private void Rotator()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.ROTATOR_KEY))
			rotatorState = rotatorState == Utilities.State.IDLE ? Utilities.State.BUSY : Utilities.State.IDLE;
		robotHardware.rotatorSystem.SetPosition(rotatorState == Utilities.State.IDLE ? Constants.getRotatorIdle() : Constants.getRotatorBusy());
	}

	private boolean planeLaunched = false;

	private void Plane()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.PLANE_COMBO) && !planeLaunched)
		{
			planeLaunched = true;
			robotHardware.droneSystem.LaunchDrone();
		}
	}

	private void Telemetry()
	{
		telemetry.addData("Robot Vel", robotHardware.mecanumDrive.updatePoseEstimate().linearVel);

		telemetry.addData("Front Left", robotHardware.mecanumDrive.leftFront.getCurrent(CurrentUnit.MILLIAMPS));
		telemetry.addData("Front Right", robotHardware.mecanumDrive.rightFront.getCurrent(CurrentUnit.MILLIAMPS));
		telemetry.addData("Back Left", robotHardware.mecanumDrive.leftBack.getCurrent(CurrentUnit.MILLIAMPS));
		telemetry.addData("Back Right", robotHardware.mecanumDrive.rightBack.getCurrent(CurrentUnit.MILLIAMPS));

		telemetry.update();
	}
}
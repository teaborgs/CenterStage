package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;
import static org.firstinspires.ftc.teamcode.Utilities.CutPower;
import static org.firstinspires.ftc.teamcode.Utilities.RestorePower;
import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities;

@TeleOp(name = "Razvan TeleOp", group = "TeleOp")
public final class RazvanTeleOp extends BaseOpMode
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

	// Input System
	private InputSystem wheelInput, armInput;

	private static final boolean INVERTED = false;
	private static final boolean DEBUG = false;

	private final static class Bindings
	{
		private final static class Wheel
		{
			private static final InputSystem.Key TURBO_KEY = new InputSystem.Key("left_bumper");
			private static final InputSystem.Key SUPPRESS_KEY = new InputSystem.Key("right_bumper");
			private static final InputSystem.Axis DRIVE_AXIS_X = new InputSystem.Axis("left_stick_x");
			private static final InputSystem.Axis DRIVE_AXIS_Y = new InputSystem.Axis("left_stick_y");
			private static final InputSystem.Axis ROTATE_AXIS_L = new InputSystem.Axis("left_trigger");
			private static final InputSystem.Axis ROTATE_AXIS_R = new InputSystem.Axis("right_trigger");
			private static final InputSystem.Key INTAKE_KEY = new InputSystem.Key("a");
			private static final InputSystem.Key INTAKE_REVERSE_KEY = new InputSystem.Key("b");
			private static final InputSystem.Key TOGGLE_PICKUP_MODE_KEY = new InputSystem.Key("x");
		}

		private final static class Arm
		{
			private static final InputSystem.BindingCombo PLANE_COMBO = new InputSystem.BindingCombo("_plane", new InputSystem.Key("left_bumper"), new InputSystem.Key("right_bumper"));
			private static final InputSystem.Key SUSPENDER_KEY = new InputSystem.Key("x");
			private static final InputSystem.Key SUSPENDER_CANCEL_KEY = new InputSystem.Key("y");
			private static final InputSystem.Key ARM_KEY = new InputSystem.Key("a");
			private static final InputSystem.Key ARM_CONFIRM_KEY = new InputSystem.Key("b");
			private static final InputSystem.Key LEVEL_2_KEY = new InputSystem.Key("dpad_up");
			private static final InputSystem.Key LEVEL_1_KEY = new InputSystem.Key("dpad_down");
		}
	}

	@Override
	protected void OnInitialize()
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

		wheelInput = new InputSystem(gamepad1);
		armInput = new InputSystem(gamepad2);

		telemetry.setMsTransmissionInterval(50);
	}

	@Override
	protected void OnRun()
	{
		Telemetry();
		UpdateMotorPowers();
		Wheels();
		Suspender();
		if (isSuspending) return;
		Leveler();
		Pickup();
		Arm();
		Plane();
	}

	// ================ Wheels ================
	private void Wheels()
	{
		boolean turbo = wheelInput.isPressed(Bindings.Wheel.TURBO_KEY);
		boolean suppress = wheelInput.isPressed(Bindings.Wheel.SUPPRESS_KEY);
		int modifier = INVERTED ? -1 : 1;
		double angle = (wheelInput.getValue(Bindings.Wheel.ROTATE_AXIS_L) - wheelInput.getValue(Bindings.Wheel.ROTATE_AXIS_R)) * (turbo ? 0.08 : suppress ? 0.032 : 0.064);
		if (INVERTED) angle *= -1;
		Vector2d wheelVel = new Vector2d(
				wheelInput.getValue(Bindings.Wheel.DRIVE_AXIS_Y) * modifier,
				wheelInput.getValue(Bindings.Wheel.DRIVE_AXIS_X) * modifier
		).times(turbo ? 1.0 : suppress ? 0.3 : 0.6);

		// Drive
		mecanumDrive.setDrivePowers(new PoseVelocity2d(wheelVel, angle));
	}

	private Utilities.PickupMode pickupMode = Utilities.PickupMode.LOAD;

	private void Pickup()
	{
		if (wheelInput.wasPressedThisFrame(Bindings.Wheel.TOGGLE_PICKUP_MODE_KEY)) {
			pickupMode = pickupMode == Utilities.PickupMode.LOAD ? Utilities.PickupMode.STACK : Utilities.PickupMode.LOAD;
			if (pickupMode == Utilities.PickupMode.STACK) intakeMotor.setMotorDisable();
			else intakeMotor.setMotorEnable();
			gamepad2.rumble(500);
		}
		if (pickupMode == Utilities.PickupMode.LOAD)
			intakeMotor.setPower(wheelInput.isPressed(Bindings.Wheel.INTAKE_KEY) ? 0.8 : wheelInput.isPressed(Bindings.Wheel.INTAKE_REVERSE_KEY) ? -0.8 : 0);
	}

	private void UpdateMotorPowers()
	{
		if (Math.abs(liftMotor1.getCurrentPosition() - liftMotor1.getTargetPosition()) > TOLERANCE)
			liftMotor1.setPower(1);
		else
			liftMotor1.setPower(0);
		if (Math.abs(liftMotor2.getCurrentPosition() - liftMotor2.getTargetPosition()) > TOLERANCE)
			liftMotor2.setPower(1);
		else
			liftMotor2.setPower(0);
		if (Math.abs(tumblerMotor.getCurrentPosition() - tumblerMotor.getTargetPosition()) > TOLERANCE)
			tumblerMotor.setPower(0.8);
		else
			tumblerMotor.setPower(0);
	}

	// ================ Arm ================

	private short liftLevel = 1;

	private void Leveler()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.LEVEL_1_KEY)) liftLevel = 1;
		else if (armInput.wasPressedThisFrame(Bindings.Arm.LEVEL_2_KEY)) liftLevel = 2;
	}

	private Utilities.State armState = Utilities.State.IDLE;
	private volatile boolean armBusy = false;
	private short stackLevel = 5;
	private boolean isAtLevel = false;
	private boolean isOutOfBounds = false;
	private boolean has2StackPixels = false;
	private boolean isWaitingForSecondInstruction = false;

	private void Arm()
	{
		if (armBusy) return;
		if (armInput.wasPressedThisFrame(Bindings.Arm.ARM_CONFIRM_KEY) && isWaitingForSecondInstruction && !has2StackPixels) {
			armBusy = true;
			rotatorServo.setPosition(Constants.Data.Rotator.IDLE);
			setTimeout(() -> {
				tumblerMotor.setTargetPosition(Constants.Data.Tumbler.LOAD);
				setTimeout(() -> {
					clawServo.setPosition(Constants.Data.Claw.IDLE);
					setTimeout(() -> {
						tumblerMotor.setTargetPosition(Constants.Data.Tumbler.IDLE);
						armBusy = false;
						isWaitingForSecondInstruction = false;
						has2StackPixels = true;
					}, 500);
				}, 500);
			}, 500);
		} else if (armInput.wasPressedThisFrame(Bindings.Arm.ARM_KEY)) {
			if (armState == Utilities.State.BUSY) { // Drop Pixel and return to Idle
				armBusy = true;
				clawServo.setPosition(Constants.Data.Claw.IDLE);
				setTimeout(() -> {
					tumblerMotor.setTargetPosition(Constants.Data.Tumbler.IDLE);
					setTimeout(() -> {
						liftMotor1.setTargetPosition(Constants.Data.Lift.PICKUP);
						liftMotor2.setTargetPosition(Constants.Data.Lift.PICKUP);
						rotatorServo.setPosition(Constants.Data.Rotator.IDLE);
						armState = Utilities.State.IDLE;
						armBusy = false;
					}, 500);
				}, 500);
			} else { // Pickup Pixel and go to Backdrop
				if (pickupMode == Utilities.PickupMode.LOAD) {
					armBusy = true;
					tumblerMotor.setTargetPosition(Constants.Data.Tumbler.LOAD);
					setTimeout(() -> {
						clawServo.setPosition(Constants.Data.Claw.BUSY);
						setTimeout(() -> {
							liftMotor1.setTargetPosition(liftLevel == 1 ? Constants.Data.Lift.LEVEL_1 : Constants.Data.Lift.LEVEL_2);
							liftMotor2.setTargetPosition(liftLevel == 1 ? Constants.Data.Lift.LEVEL_1 : Constants.Data.Lift.LEVEL_2);
							tumblerMotor.setTargetPosition(Constants.Data.Tumbler.BACKDROP);
							setTimeout(() -> {
								rotatorServo.setPosition(Constants.Data.Rotator.BUSY);
								armBusy = false;
								armState = Utilities.State.BUSY;
							}, 200 * liftLevel);
						}, 500);
					}, 500);
				} else if (!isOutOfBounds) {
					if (!isAtLevel && !isWaitingForSecondInstruction) {
						armBusy = true;
						rotatorServo.setPosition(Constants.Data.Rotator.BUSY);
						setTimeout(() -> {
							tumblerMotor.setTargetPosition(Constants.Data.Tumbler.STACK_POSES[stackLevel - 1] - 80);
							isAtLevel = true;
							armBusy = false;
						}, 100);
					} else if (isAtLevel && !isWaitingForSecondInstruction) {
						armBusy = true;
						tumblerMotor.setTargetPosition(Constants.Data.Tumbler.STACK_POSES[stackLevel - 1]);
						setTimeout(() -> {
							clawServo.setPosition(Constants.Data.Claw.BUSY);
							setTimeout(() -> {
								tumblerMotor.setTargetPosition(Constants.Data.Tumbler.BACKDROP);
								armBusy = false;
								isAtLevel = false;
								isWaitingForSecondInstruction = true;
								stackLevel--;
								if (stackLevel == 0) isOutOfBounds = true;
							}, 500);
						}, 300);
					} else if (!isAtLevel) {
						armBusy = true;
						liftMotor1.setTargetPosition(liftLevel == 1 ? Constants.Data.Lift.LEVEL_1 : Constants.Data.Lift.LEVEL_2);
						liftMotor2.setTargetPosition(liftLevel == 1 ? Constants.Data.Lift.LEVEL_1 : Constants.Data.Lift.LEVEL_2);
						armBusy = false;
						isWaitingForSecondInstruction = false;
						armState = Utilities.State.BUSY;
					}
				}
			}
		}
	}

	private volatile boolean planeLaunched = false;

	private void Plane()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.PLANE_COMBO) && !planeLaunched) {
			planeLaunched = true;
			planeLevelServo.setPosition(Constants.Data.Plane.Leveler.BUSY);
			setTimeout(() -> {
				planeReleaseServo.setPosition(Constants.Data.Plane.Releaser.BUSY);
				setTimeout(() -> {
					planeLevelServo.setPosition(Constants.Data.Plane.Leveler.IDLE);
					planeReleaseServo.setPosition(Constants.Data.Plane.Releaser.IDLE);
				}, 200);
			}, 300);
		}
	}

	private boolean robotSuspended = false;
	private boolean isSuspending = false;

	private void Suspender()
	{
		// Locker
		if (robotSuspended && liftMotor1.getCurrentPosition() <= TOLERANCE && liftMotor2.getCurrentPosition() <= TOLERANCE) {
			setTimeout(() -> {
				lockerServo.setPosition(Constants.Data.Locker.BUSY);
			}, 500);
		}

		// Lift
		if (armInput.wasPressedThisFrame(Bindings.Arm.SUSPENDER_CANCEL_KEY) && isSuspending && !robotSuspended) {
			isSuspending = false;
			liftMotor2.setTargetPosition(Constants.Data.Suspender.IDLE);
			liftMotor1.setTargetPosition(Constants.Data.Suspender.IDLE);
			RestorePower(tumblerMotor, intakeMotor);
		} else if (armInput.wasPressedThisFrame(Bindings.Arm.SUSPENDER_KEY)) {
			if (liftMotor1.getCurrentPosition() <= TOLERANCE && liftMotor2.getCurrentPosition() <= TOLERANCE) {
				CutPower(tumblerMotor, intakeMotor);
				isSuspending = true;
				liftMotor2.setTargetPosition(Constants.Data.Suspender.SUSPEND);
				liftMotor1.setTargetPosition(Constants.Data.Suspender.SUSPEND);
			}
			if (liftMotor1.getCurrentPosition() >= Constants.Data.Suspender.SUSPEND - TOLERANCE && liftMotor2.getCurrentPosition() >= Constants.Data.Suspender.SUSPEND - TOLERANCE) {
				liftMotor2.setTargetPosition(Constants.Data.Suspender.LOCK);
				liftMotor1.setTargetPosition(Constants.Data.Suspender.LOCK);
				robotSuspended = true;
			}
		}
	}

	@Override
	protected void OnStop()
	{
		if (robotSuspended) return;
		tumblerMotor.setTargetPosition(Constants.Data.Tumbler.LOAD);
		while (tumblerMotor.getCurrentPosition() > Constants.Data.Tumbler.LOAD + TOLERANCE)
			tumblerMotor.setPower(0.8);
	}

	private void Telemetry()
	{
		if (isOutOfBounds && pickupMode == Utilities.PickupMode.STACK) {
			telemetry.clearAll();
			telemetry.addLine("[WARN] Robot is out of bounds. Please change pickup mode!");
			telemetry.update();
			return;
		}

		if (isSuspending) {
			telemetry.clearAll();
			if (!robotSuspended)
				telemetry.addLine("[WARN] Currently in suspension mode. Press Y to cancel.");
			telemetry.addData("[INFO] Robot Suspended:", robotSuspended ? "Yes" : "No");
			telemetry.update();
			return;
		}

		telemetry.addData("[INFO] Pickup Mode", pickupMode.toString());
		telemetry.addData("[INFO] Arm State", armState.toString());
		telemetry.addData("[INFO] Lift Level", liftLevel);
		telemetry.addData("[INFO] Plane Launched", planeLaunched ? "Yes" : "No");

		if (DEBUG) {
			telemetry.addLine();
			telemetry.addData("[DEBUG] Lift 1", liftMotor1.getCurrentPosition());
			telemetry.addData("[DEBUG] Lift 2", liftMotor2.getCurrentPosition());
			telemetry.addData("[DEBUG] Lift 1 Target", liftMotor1.getTargetPosition());
			telemetry.addData("[DEBUG] Lift 2 Target", liftMotor2.getTargetPosition());
			telemetry.addData("[DEBUG] Tumbler", tumblerMotor.getCurrentPosition());
			telemetry.addData("[DEBUG] Tumbler Target", tumblerMotor.getTargetPosition());
			telemetry.addData("[DEBUG] Rotator", rotatorServo.getPosition());
			telemetry.addData("[DEBUG] Claw", clawServo.getPosition());
			telemetry.addData("[DEBUG] Locker", lockerServo.getPosition());
			telemetry.addData("[DEBUG] Plane Level", planeLevelServo.getPosition());
			telemetry.addData("[DEBUG] Plane Release", planeReleaseServo.getPosition());
			telemetry.addData("[DEBUG] Arm Busy", armBusy);
			telemetry.addData("[DEBUG] Arm Waiting", isWaitingForSecondInstruction);
			telemetry.addData("[DEBUG] Arm At Level", isAtLevel);
			telemetry.addData("[DEBUG] Arm Out Of Bounds", isOutOfBounds);
			telemetry.addData("[DEBUG] Arm Has 2 Stack Pixels", has2StackPixels);
			telemetry.addData("[DEBUG] Stack Level", stackLevel);
		}

		telemetry.update();
	}
}
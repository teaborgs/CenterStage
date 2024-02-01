package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;
import static org.firstinspires.ftc.teamcode.Constants.getSuspenderSuspend;
import static org.firstinspires.ftc.teamcode.Utilities.CutPower;
import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Alex TeleOp", group = "TeleOp")
public final class AlexTeleOp extends BaseOpMode
{
	private RobotHardware robotHardware;
	private InputSystem wheelInput, armInput;

	private final static class Bindings
	{
		private final static class Wheel
		{
			private static final InputSystem.Key TURBO_KEY = new InputSystem.Key("right_bumper");
			private static final InputSystem.Key SUPPRESS_KEY = new InputSystem.Key("left_bumper");
			private static final InputSystem.Axis DRIVE_AXIS_X = new InputSystem.Axis("left_stick_x");
			private static final InputSystem.Axis DRIVE_AXIS_Y = new InputSystem.Axis("left_stick_y");
			private static final InputSystem.Axis ROTATE_AXIS_L = new InputSystem.Axis("left_trigger");
			private static final InputSystem.Axis ROTATE_AXIS_R = new InputSystem.Axis("right_trigger");
			private static final InputSystem.Key INTAKE_KEY = new InputSystem.Key("a");
			private static final InputSystem.Key INTAKE_REVERSE_KEY = new InputSystem.Key("b");
			private static final InputSystem.Key INTAKE_NO_HELP_KEY = new InputSystem.Key("x");
			private static final InputSystem.Key GRAB_STACK_KEY = new InputSystem.Key("y");
		}

		private final static class Arm
		{
			private static final InputSystem.BindingCombo PLANE_COMBO = new InputSystem.BindingCombo("_plane", new InputSystem.Key("left_bumper"), new InputSystem.Key("right_bumper"));
			private static final InputSystem.Key SUSPENDER_KEY = new InputSystem.Key("x");
			private static final InputSystem.Key SUSPENDER_CANCEL_KEY = new InputSystem.Key("y");
			private static final InputSystem.Key ARM_KEY = new InputSystem.Key("a");
			private static final InputSystem.Key LEVEL_1_KEY = new InputSystem.Key("dpad_down");
			private static final InputSystem.Key LEVEL_2_KEY = new InputSystem.Key("dpad_up");
			private static final InputSystem.Key LEVEL_3_KEY = new InputSystem.Key("dpad_left");
			private static final InputSystem.Key LEVEL_4_KEY = new InputSystem.Key("dpad_right");
			private static final InputSystem.Key LEVEL_5_KEY = new InputSystem.Key("back");
			private static final InputSystem.Key RESET_MODE_KEY = new InputSystem.Key("y");
		}
	}

	@Override
	protected void OnInitialize()
	{
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2); // This is to make sure the robot is selected before the init is done
		Constants.Init();
		robotHardware = new RobotHardware(hardwareMap);

		wheelInput = new InputSystem(gamepad1);
		armInput = new InputSystem(gamepad2);

		telemetry.setMsTransmissionInterval(50);
	}

	@Override
	protected void OnRun()
	{
		Telemetry();
		UpdateMotorPowers();
		Suspender();
		Antenna();
		Wheels();
		if (suspending) return;
		ManualReset();
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
		double angle = (wheelInput.getValue(Bindings.Wheel.ROTATE_AXIS_R) - wheelInput.getValue(Bindings.Wheel.ROTATE_AXIS_L)) * (turbo ? 0.1 : suppress ? 0.03 : 0.08);
		Vector2d wheelVel = new Vector2d(
				wheelInput.getValue(Bindings.Wheel.DRIVE_AXIS_Y),
				wheelInput.getValue(Bindings.Wheel.DRIVE_AXIS_X)
		).times(turbo ? 1.0 : suppress ? 0.3 : 0.8);
		robotHardware.mecanumDrive.setDrivePowers(new PoseVelocity2d(wheelVel.times(-1), -angle));
	}

	private void Pickup()
	{
		if (!wheelInput.isPressed(Bindings.Wheel.GRAB_STACK_KEY) && antennaPress.seconds() > Constants.getAntennaIntakeRunTime()) {
			robotHardware.intakeMotor.setPower(wheelInput.isPressed(Bindings.Wheel.INTAKE_KEY) ? Constants.getIntakeMaxPower() : wheelInput.isPressed(Bindings.Wheel.INTAKE_NO_HELP_KEY) ? Constants.getIntakeMaxPower() : wheelInput.isPressed(Bindings.Wheel.INTAKE_REVERSE_KEY) ? -Constants.getIntakeMaxPower() : 0);
			if (wheelInput.isPressed(Bindings.Wheel.INTAKE_KEY)) robotHardware.antennaServo.setPosition(Constants.getAntennaGuide());
			else if (!wheelInput.isPressed(Bindings.Wheel.GRAB_STACK_KEY)) robotHardware.antennaServo.setPosition(Constants.getAntennaIdle());
		}
	}

	private final ElapsedTime antennaPress = new ElapsedTime();

	private void Antenna()
	{
		if (wheelInput.isPressed(Bindings.Wheel.GRAB_STACK_KEY)) {
			antennaPress.reset();
			robotHardware.antennaServo.setPosition(Constants.getAntennaGrab());
			robotHardware.intakeMotor.setPower(Constants.getIntakeMaxPower());
		} else {
			if (!wheelInput.isPressed(Bindings.Wheel.INTAKE_KEY))
				robotHardware.antennaServo.setPosition(Constants.getAntennaIdle());

			if (antennaPress.time(TimeUnit.SECONDS) > Constants.getAntennaIntakeRunTime())
				robotHardware.intakeMotor.setPower(0);
		}
	}

	private void UpdateMotorPowers()
	{
		if (robotSuspended || suspending)
			robotHardware.liftMotor1.setPower(Constants.getLiftSuspendPower());
		else if (Math.abs(robotHardware.liftMotor1.getCurrentPosition() - robotHardware.liftMotor1.getTargetPosition()) > TOLERANCE)
			robotHardware.liftMotor1.setPower(Constants.getLiftNormalPower());
		else
			robotHardware.liftMotor1.setPower(0.05);

		if (robotSuspended || suspending)
			robotHardware.liftMotor2.setPower(Constants.getLiftSuspendPower());
		else if (Math.abs(robotHardware.liftMotor2.getCurrentPosition() - robotHardware.liftMotor2.getTargetPosition()) > TOLERANCE)
			robotHardware.liftMotor2.setPower(Constants.getLiftNormalPower());
		else
			robotHardware.liftMotor2.setPower(0.05);

		if (inResetMode) return;
		if (Math.abs(robotHardware.tumblerMotor.getCurrentPosition() - robotHardware.tumblerMotor.getTargetPosition()) > TOLERANCE / 2)
			robotHardware.tumblerMotor.setPower(1);
		else
			robotHardware.tumblerMotor.setPower(0.05);
	}


	// ================ Arm ================

	private short liftLevel = 1;

	private void Leveler()
	{
		int initialLevel = liftLevel;
		if (armInput.wasPressedThisFrame(Bindings.Arm.LEVEL_1_KEY)) liftLevel = 1;
		else if (armInput.wasPressedThisFrame(Bindings.Arm.LEVEL_2_KEY)) liftLevel = 2;
		else if (armInput.wasPressedThisFrame(Bindings.Arm.LEVEL_3_KEY)) liftLevel = 3;
		else if (armInput.wasPressedThisFrame(Bindings.Arm.LEVEL_4_KEY)) liftLevel = 4;
		else if (armInput.wasPressedThisFrame(Bindings.Arm.LEVEL_5_KEY)) liftLevel = 5;
		if (initialLevel != liftLevel && armState == Utilities.State.BUSY) {
			robotHardware.liftMotor1.setTargetPosition(Constants.getLiftLevels()[liftLevel]);
			robotHardware.liftMotor2.setTargetPosition(Constants.getLiftLevels()[liftLevel]);
		}
	}

	private boolean inResetMode = false;

	private void ManualReset()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.RESET_MODE_KEY)) inResetMode = !inResetMode;
		if (!inResetMode) return;

		robotHardware.tumblerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		robotHardware.tumblerMotor.setPower(-0.5);
		robotHardware.rotatorServo.setPosition(Constants.getRotatorIdle());
		robotHardware.clawServo.setPosition(Constants.getClawIdle());

		if (robotHardware.tumblerMotor.getCurrent(CurrentUnit.MILLIAMPS) > Constants.getTumblerMaxCurrent()) {
			robotHardware.tumblerMotor.setPower(0);
			robotHardware.tumblerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
			robotHardware.tumblerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
			robotHardware.tumblerMotor.setDirection(DcMotorEx.Direction.REVERSE);
			robotHardware.tumblerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
			robotHardware.tumblerMotor.setTargetPosition(Constants.getTumblerIdle());
			robotHardware.tumblerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
			new Thread()
			{
				@Override
				public void run()
				{
					while (robotHardware.tumblerMotor.getCurrentPosition() <= 90) {
						// hehe empty while go brrr
					};
					robotHardware.tumblerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
					robotHardware.tumblerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
					robotHardware.tumblerMotor.setDirection(DcMotorEx.Direction.REVERSE);
					robotHardware.tumblerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
					robotHardware.tumblerMotor.setTargetPosition(Constants.getTumblerIdle());
					robotHardware.tumblerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
				}
			}.start();
			inResetMode = false;
		}
	}

	private Utilities.State armState = Utilities.State.IDLE;
	private volatile boolean armBusy = false;

	private void Arm()
	{
		if (armBusy) return;
		if (armInput.wasPressedThisFrame(Bindings.Arm.ARM_KEY)) {
			if (armState == Utilities.State.BUSY) { // Drop Pixel and return to Idle
				armBusy = true;
				robotHardware.clawServo.setPosition(Constants.getClawIdle());
				setTimeout(() -> {
					robotHardware.tumblerMotor.setTargetPosition(Constants.getTumblerIdle());
					setTimeout(() -> {
						robotHardware.liftMotor1.setTargetPosition(Constants.getLiftLevels()[0]);
						robotHardware.liftMotor2.setTargetPosition(Constants.getLiftLevels()[0]);
						robotHardware.rotatorServo.setPosition(Constants.getRotatorIdle());
						armState = Utilities.State.IDLE;
						armBusy = false;
					}, 500);
				}, 750);
			} else {
				armBusy = true;
				robotHardware.tumblerMotor.setTargetPosition(Constants.getTumblerLoad());
				setTimeout(() -> {
					robotHardware.clawServo.setPosition(Constants.getClawBusy());
					setTimeout(() -> {
						robotHardware.liftMotor1.setTargetPosition(Constants.getLiftLevels()[liftLevel]);
						robotHardware.liftMotor2.setTargetPosition(Constants.getLiftLevels()[liftLevel]);
						robotHardware.tumblerMotor.setTargetPosition(Constants.getTumblerBackdrop());
						setTimeout(() -> {
							robotHardware.rotatorServo.setPosition(Constants.getRotatorBusy());
							armBusy = false;
							armState = Utilities.State.BUSY;
						}, 200 * liftLevel);
					}, 500);
				}, 500);
			}
		}
	}

	private volatile boolean planeLaunched = false;

	private void Plane()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.PLANE_COMBO) && !planeLaunched) {
			planeLaunched = true;
			robotHardware.planeLevelServo.setPosition(Constants.getPlaneLevelerBusy());
			setTimeout(() -> {
				robotHardware.planeShooterServo.setPosition(Constants.getPlaneShooterBusy());
				setTimeout(() -> {
					robotHardware.planeLevelServo.setPosition(Constants.getPlaneLevelerIdle());
					robotHardware.planeShooterServo.setPosition(Constants.getPlaneShooterIdle());
				}, 200);
			}, 300);
		}
	}

	private boolean robotSuspended = false;
	private boolean suspending = false;

	private void Suspender()
	{
		// Lift
		if (armInput.wasPressedThisFrame(Bindings.Arm.SUSPENDER_CANCEL_KEY) && suspending && !robotSuspended) {
			suspending = false;
			robotHardware.liftMotor2.setTargetPosition(Constants.getSuspenderIdle());
			robotHardware.liftMotor1.setTargetPosition(Constants.getSuspenderIdle());
		} else if (armInput.wasPressedThisFrame(Bindings.Arm.SUSPENDER_KEY)) {
			if (!suspending) {
				suspending = true;
				robotHardware.liftMotor2.setTargetPosition(getSuspenderSuspend());
				robotHardware.liftMotor1.setTargetPosition(Constants.getSuspenderSuspend());
			} else if (robotHardware.liftMotor1.getCurrentPosition() >= Constants.getSuspenderSuspend() - TOLERANCE && robotHardware.liftMotor2.getCurrentPosition() >= Constants.getSuspenderSuspend() - TOLERANCE) {
				robotHardware.liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				robotHardware.liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				robotHardware.liftMotor2.setTargetPosition(Constants.getSuspenderSuspend() - 800);
				robotHardware.liftMotor1.setTargetPosition(Constants.getSuspenderSuspend() - 800);
				CutPower(robotHardware.mecanumDrive.leftBack, robotHardware.mecanumDrive.rightBack, robotHardware.mecanumDrive.leftFront, robotHardware.mecanumDrive.rightFront);
				CutPower(robotHardware.intakeMotor, robotHardware.tumblerMotor);
				CutPower(robotHardware.planeLevelServo, robotHardware.planeShooterServo);
				robotSuspended = true;
			}
		}
	}

	private void Telemetry()
	{
		// Warn the user when in manual override mode
		if (inResetMode) {
			telemetry.clearAll();
			telemetry.addLine("[WARN] Robot is in manual override mode. To exit press Y, to save changes press B");
			telemetry.update();
			return;
		}

		// Warn the user if the robot is in suspension mode
		if (suspending) {
			telemetry.clearAll();
			if (!robotSuspended)
				telemetry.addLine("[WARN] Currently in suspension mode. Press Y to cancel.");
			telemetry.addData("[INFO] Robot Suspended:", robotSuspended ? "Yes" : "No");
			telemetry.update();
			return;
		}

		telemetry.addData("[INFO] Arm State", armState.toString());
		telemetry.addData("[INFO] Lift Level", liftLevel);
		telemetry.addData("[INFO] Plane Launched", planeLaunched ? "Yes" : "No");

		if (Globals.IsDebugging()) {
			telemetry.addLine();
			telemetry.addData("[DEBUG] Antenna Pos", robotHardware.antennaServo.getPosition());
			telemetry.addData("[DEBUG] Last Antenna press", antennaPress.seconds());
			telemetry.addData("[DEBUG] Distance Sensor: ", robotHardware.distanceSensor.getDistance(DistanceUnit.CM));
			telemetry.addData("[DEBUG] Lift 1", robotHardware.liftMotor1.getCurrentPosition());
			telemetry.addData("[DEBUG] Lift 2", robotHardware.liftMotor2.getCurrentPosition());
			telemetry.addData("[DEBUG] Lift 1 Target", robotHardware.liftMotor1.getTargetPosition());
			telemetry.addData("[DEBUG] Lift 2 Target", robotHardware.liftMotor2.getTargetPosition());
			telemetry.addData("[DEBUG] Lift 1 Power", robotHardware.liftMotor2.getPower());
			telemetry.addData("[DEBUG] Lift 2 Power", robotHardware.liftMotor2.getPower());
			telemetry.addData("[DEBUG] Tumbler", robotHardware.tumblerMotor.getCurrentPosition());
			telemetry.addData("[DEBUG] Tumbler Target", robotHardware.tumblerMotor.getTargetPosition());
			telemetry.addData("[DEBUG] Tumbler Power", robotHardware.tumblerMotor.getPower());
			telemetry.addData("[DEBUG] Intake Power", robotHardware.intakeMotor.getPower());
			telemetry.addData("[DEBUG] Rotator", robotHardware.rotatorServo.getPosition());
			telemetry.addData("[DEBUG] Claw", robotHardware.clawServo.getPosition());
			telemetry.addData("[DEBUG] Plane Level", robotHardware.planeLevelServo.getPosition());
			telemetry.addData("[DEBUG] Plane Release", robotHardware.planeShooterServo.getPosition());
			telemetry.addData("[DEBUG] Arm Busy", armBusy);
		}
		telemetry.update();
	}
}
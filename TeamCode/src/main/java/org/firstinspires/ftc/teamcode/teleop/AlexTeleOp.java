package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;
import static org.firstinspires.ftc.teamcode.Utilities.CreateVideoFolder;
import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import android.os.Environment;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.opencv.VideoWriterPipeline;

import java.util.Date;
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

	private VideoWriterPipeline videoWriterPipeline;

	@Override
	protected void OnInitialize()
	{
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2); // This is to make sure the robot is selected before the init is done
		Constants.Init();
		robotHardware = new RobotHardware(hardwareMap);

		CreateVideoFolder();
		videoWriterPipeline = new VideoWriterPipeline(Environment.getExternalStorageDirectory().getAbsolutePath() + "/secret/alex/" + new Date().toString().replace(' ', '_') + ".mp4");
		robotHardware.camera.setPipeline(videoWriterPipeline);

		wheelInput = new InputSystem(gamepad1);
		armInput = new InputSystem(gamepad2);
	}

	@Override
	protected void OnRun()
	{
		Telemetry();
		UpdateMotorPowers();
		ManualReset();
		Suspender();
		Antenna();
		Wheels();
		if (suspending || robotSuspended) return;
		Leveler();
		Pickup();
		Arm();
		Drone();
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
		if (!wheelInput.isPressed(Bindings.Wheel.GRAB_STACK_KEY) && antennaPress.seconds() > Constants.getAntennaRunTime()) {

			robotHardware.intakeSystem.SetIntakePower((wheelInput.isPressed(Bindings.Wheel.INTAKE_KEY) || wheelInput.isPressed(Bindings.Wheel.INTAKE_NO_HELP_KEY) || antennaState == Utilities.State.BUSY) ? Constants.getIntakeMaxPower() : wheelInput.isPressed(Bindings.Wheel.INTAKE_REVERSE_KEY) ? -Constants.getIntakeMaxPower() : 0);

			if (antennaState == Utilities.State.BUSY) return;
			if (wheelInput.isPressed(Bindings.Wheel.INTAKE_KEY))
				robotHardware.intakeSystem.SetAntennaPosition(Constants.getAntennaGuide());
			else robotHardware.intakeSystem.SetAntennaPosition(Constants.getAntennaIdle());
		}
	}

	private final ElapsedTime antennaPress = new ElapsedTime();
	private Utilities.State antennaState = Utilities.State.IDLE;

	private void Antenna()
	{
		if (wheelInput.isPressed(Bindings.Wheel.GRAB_STACK_KEY) && antennaState == Utilities.State.IDLE) {
			antennaState = Utilities.State.BUSY;
			antennaPress.reset();
			robotHardware.intakeSystem.SetAntennaPosition(Constants.getAntennaGrab());
			robotHardware.intakeSystem.SetIntakePower(Constants.getIntakeMaxPower());
		} else if (antennaPress.time(TimeUnit.MILLISECONDS) / 1000d > Constants.getAntennaRunTime() && !wheelInput.isPressed(Bindings.Wheel.INTAKE_KEY) && !wheelInput.isPressed(Bindings.Wheel.INTAKE_NO_HELP_KEY)) {
			antennaState = Utilities.State.IDLE;
			robotHardware.intakeSystem.SetAntennaPosition(Constants.getAntennaIdle());
			robotHardware.intakeSystem.SetIntakePower(0);
		}
	}

	private void UpdateMotorPowers()
	{
		if (robotSuspended || suspending)
			robotHardware.liftSystem.SetPower(Constants.getLiftSuspendPower());
		else if (Math.abs(robotHardware.liftSystem.GetCurrentPosition() - robotHardware.liftSystem.GetTargetPosition()) > TOLERANCE)
			robotHardware.liftSystem.SetPower(Constants.getLiftNormalPower());
		else
			robotHardware.liftSystem.SetPower(0.05);

		if (inResetMode) return;
		if (Math.abs(robotHardware.tumblerSystem.GetCurrentPosition() - robotHardware.tumblerSystem.GetTargetPosition()) > TOLERANCE / 2d)
			robotHardware.tumblerSystem.SetPower(1);
		else
			robotHardware.tumblerSystem.SetPower(0.05);
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
		if (initialLevel != liftLevel && armState == Utilities.State.BUSY)
			robotHardware.liftSystem.SetTargetPosition(Constants.getLiftLevels()[liftLevel]);
	}

	private boolean inResetMode = false;

	private void ManualReset()
	{
		if (armInput.wasPressedThisFrame(Bindings.Arm.RESET_MODE_KEY) && !suspending && !robotSuspended)
			inResetMode = !inResetMode;
		if (!inResetMode) return;

		robotHardware.clawSystem.SetPosition(Constants.getClawIdle());
		robotHardware.rotatorSystem.SetPosition(Constants.getRotatorIdle());
		robotHardware.tumblerSystem.SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		robotHardware.tumblerSystem.SetPower(-0.5);

		if (robotHardware.tumblerSystem.GetCurrent(CurrentUnit.MILLIAMPS) > Constants.getTumblerMaxCurrent()) {
			robotHardware.tumblerSystem.SetPower(0);
			robotHardware.tumblerSystem.Init();
			new Thread()
			{
				@Override
				public void run()
				{
					while (robotHardware.tumblerSystem.GetCurrentPosition() <= 90) {
						// hehe empty while go brrr
					}
					robotHardware.tumblerSystem.Init();
				}
			}.start();
			inResetMode = false;
		}
	}

	private Utilities.State armState = Utilities.State.IDLE;
	private volatile boolean armBusy = false;

	private void Arm()
	{
		if (armBusy || !armInput.wasPressedThisFrame(Bindings.Arm.ARM_KEY)) return;
		armBusy = true;
		if (armState == Utilities.State.BUSY) { // Drop Pixel and return to Idle
			robotHardware.clawSystem.SetPosition(Constants.getClawIdle());
			setTimeout(() -> {
				robotHardware.tumblerSystem.SetTargetPosition(Constants.getTumblerIdle());
				setTimeout(() -> {
					robotHardware.rotatorSystem.SetPosition(Constants.getRotatorIdle());
					robotHardware.liftSystem.SetTargetPosition(Constants.getLiftLevels()[0]);
					armState = Utilities.State.IDLE;
					armBusy = false;
				}, 400);
			}, 750);
		} else {
			robotHardware.rotatorSystem.SetPosition(Constants.getRotatorIdle());
			robotHardware.tumblerSystem.SetTargetPosition(Constants.getTumblerLoad());
			setTimeout(() -> {
				robotHardware.clawSystem.SetPosition(Constants.getClawBusy());
				setTimeout(() -> {
					robotHardware.liftSystem.SetTargetPosition(Constants.getLiftLevels()[liftLevel]);
					robotHardware.tumblerSystem.SetTargetPosition(Constants.getTumblerBackdrop());
					setTimeout(() -> {
						robotHardware.rotatorSystem.SetPosition(Constants.getRotatorBusy());
						armBusy = false;
						armState = Utilities.State.BUSY;
					}, 200);
				}, 500);
			}, 500);
		}
	}

	private boolean droneLaunched = false;

	private void Drone()
	{
		if (!armInput.wasPressedThisFrame(Bindings.Arm.PLANE_COMBO) || droneLaunched) return;
		droneLaunched = true;
		robotHardware.droneSystem.LaunchDrone();
	}

	private boolean robotSuspended = false;
	private boolean suspending = false;
	private final ElapsedTime suspendedTime = new ElapsedTime();

	private void Suspender()
	{
		// Lift
		if (armInput.wasPressedThisFrame(Bindings.Arm.SUSPENDER_CANCEL_KEY) && suspending && !robotSuspended && !inResetMode) {
			suspending = false;
			robotHardware.liftSystem.SetTargetPosition(Constants.getLiftSuspenderIdle());
		} else if (armInput.wasPressedThisFrame(Bindings.Arm.SUSPENDER_KEY)) {
			if (!robotSuspended) {
				if (!suspending) {
					suspending = true;
					robotHardware.liftSystem.SetTargetPosition(Constants.getLiftSuspendLevel());
				} else if (robotHardware.liftSystem.GetCurrentPosition() >= Constants.getLiftSuspendLevel() - TOLERANCE) {
					robotHardware.liftSystem.SetTargetPosition(Constants.getLiftSuspendLevel() - 800);
					robotSuspended = true;
					suspendedTime.reset();
				}
			} else if (suspendedTime.seconds() > 5) {
				robotHardware.liftSystem.SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				robotHardware.liftSystem.SetPower(0.5);
				setTimeout(() -> {
					robotHardware.liftSystem.SetPower(0);
					robotHardware.liftSystem.Init();
				}, 1);
			}
		}
	}

	private void Telemetry()
	{
		// Warn the user when in manual override mode
		if (inResetMode) {
			telemetry.clearAll();
			telemetry.addLine("[WARN] Robot is in reset mode. To exit press Y, to save changes press B");
			telemetry.update();
			return;
		}

		// Warn the user if the robot is in suspension mode
		if (suspending) {
			telemetry.clearAll();
			if (!robotSuspended)
				telemetry.addLine("[WARN] Currently in suspension mode. Press Y to cancel.");
			telemetry.addData("[INFO] Robot Suspended:", robotSuspended ? "Yes" : "No");
			if (robotSuspended) {
				telemetry.addData("[INFO] Time since suspend: ", suspendedTime.seconds());
				if (suspendedTime.seconds() > 5)
					telemetry.addLine("Press the suspend key again to lower robot");
			}
			telemetry.update();
			return;
		}

		telemetry.addData("[INFO] Arm State", armState.toString());
		telemetry.addData("[INFO] Lift Level", liftLevel);
		telemetry.addData("[INFO] Plane Launched", droneLaunched ? "Yes" : "No");

		if (Globals.IsDebugging()) {
			telemetry.addLine();
			telemetry.addData("[DEBUG] Last Antenna Press", antennaPress.seconds());
			telemetry.addData("[DEBUG] Distance Sensor: ", robotHardware.distanceSensor.getDistance(DistanceUnit.CM));
			telemetry.addData("[DEBUG] Lift Position", robotHardware.liftSystem.GetCurrentPosition());
			telemetry.addData("[DEBUG] Lift Target", robotHardware.liftSystem.GetTargetPosition());
			telemetry.addData("[DEBUG] Lift Power", robotHardware.liftSystem.GetPower());
			telemetry.addData("[DEBUG] Tumbler Position", robotHardware.tumblerSystem.GetCurrentPosition());
			telemetry.addData("[DEBUG] Tumbler Target", robotHardware.tumblerSystem.GetTargetPosition());
			telemetry.addData("[DEBUG] Tumbler Power", robotHardware.tumblerSystem.GetPower());
			telemetry.addData("[DEBUG] Rotator", robotHardware.rotatorSystem.GetCurrentPosition());
			telemetry.addData("[DEBUG] Claw", robotHardware.clawSystem.GetCurrentPosition());
			telemetry.addData("[DEBUG] Drone Level", robotHardware.droneSystem.GetLevelerPosition());
			telemetry.addData("[DEBUG] Drone Release", robotHardware.droneSystem.GetShooterPosition());
			telemetry.addData("[DEBUG] Antenna", robotHardware.intakeSystem.GetAntennaPosition());
			telemetry.addData("[DEBUG] Arm Busy", armBusy);
		}
		telemetry.update();
	}

	@Override
	protected void OnStart()
	{
		videoWriterPipeline.startRecording();
	}

	@Override
	protected void OnStop()
	{
		videoWriterPipeline.stopRecording();
	}
}
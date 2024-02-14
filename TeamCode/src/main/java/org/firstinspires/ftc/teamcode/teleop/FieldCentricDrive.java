package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Field Centric Drive", group = "! TeleOp")
public class FieldCentricDrive extends BaseOpMode
{
	private RobotHardware robotHardware;
	private InputSystem wheelInput;

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
		}
	}

	@Override
	protected void OnInitialize()
	{
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2); // This is to make sure the robot is selected before the init is done
		Constants.Init();
		robotHardware = new RobotHardware(hardwareMap);
		wheelInput = new InputSystem(gamepad1);
	}

	@Override
	protected void OnRun()
	{
		Wheels();
		Telemetry();
	}

	// Field centric drive
	private void Wheels()
	{
		boolean turbo = wheelInput.isPressed(Bindings.Wheel.TURBO_KEY);
		boolean suppress = wheelInput.isPressed(Bindings.Wheel.SUPPRESS_KEY);
		double angle = (wheelInput.getValue(Bindings.Wheel.ROTATE_AXIS_R) - wheelInput.getValue(Bindings.Wheel.ROTATE_AXIS_L)) * (turbo ? 0.1 : suppress ? 0.03 : 0.08);
		double heading = robotHardware.mecanumDrive.imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

		double x = wheelInput.getValue(Bindings.Wheel.DRIVE_AXIS_Y);
		double y = wheelInput.getValue(Bindings.Wheel.DRIVE_AXIS_X);

		Vector2d wheelVel = new Vector2d(
				x * Math.cos(-heading) - y * Math.sin(-heading),
				x * Math.sin(-heading) + y * Math.cos(-heading)
		).times(turbo ? 1.0 : suppress ? 0.3 : 0.8);


		robotHardware.mecanumDrive.setDrivePowers(new PoseVelocity2d(wheelVel.times(-1), -angle));
	}

	private void Telemetry()
	{
		telemetry.update();
	}
}

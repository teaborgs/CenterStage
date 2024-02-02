package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;

/*
 * Test two motors by controlling them with the right stick
 */

@TeleOp(name = "Two Motor Test", group = "Testing")
public class TwoMotorTest extends BaseOpMode
{
	private DcMotorEx m1, m2;

	private final InputSystem.Axis MOTOR_KEY = new InputSystem.Axis("right_stick_y");
	private InputSystem input;

	@Override
	protected void OnInitialize()
	{
		input = new InputSystem(gamepad1);
		m1 = hardwareMap.get(DcMotorEx.class, "slot0");
		m2 = hardwareMap.get(DcMotorEx.class, "slot1");
		m1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		m2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
	}

	@Override
	protected void OnRun()
	{
		m1.setPower(input.getValue(MOTOR_KEY));
		m2.setPower(input.getValue(MOTOR_KEY));
		telemetry.addData("[DEBUG] Motor 1 Power", m1.getPower());
		telemetry.addData("[DEBUG] Motor 2 Power", m2.getPower());
		telemetry.update();
	}
}
package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;

/*
 * Test a single motor by controlling it with the right stick
 */
@TeleOp(name = "Single Motor Test", group = "Testing")
public class SingleMotorTest extends BaseOpMode
{
	private final InputSystem.Axis MOTOR_KEY = new InputSystem.Axis("right_stick_y");
	private InputSystem input;
	private DcMotorEx motor;

	@Override
	protected void OnInitialize()
	{
		motor = hardwareMap.get(DcMotorEx.class, "slot0");
		input = new InputSystem(gamepad1);
	}

	@Override
	protected void OnRun()
	{
		motor.setPower(input.getValue(MOTOR_KEY));
	}
}
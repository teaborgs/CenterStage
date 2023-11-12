package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.BaseOpMode;

/*
 * Test the encoder value of a motor
 */
@TeleOp(name = "Value Test", group = "Testing")
public class ValueTest extends BaseOpMode
{
	private DcMotorEx motor;

	@Override
	protected void OnInitialize()
	{
		motor = hardwareMap.get(DcMotorEx.class, "slot7"); // Tumbler Motor
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setDirection(DcMotorSimple.Direction.REVERSE);
		motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

	@Override
	protected void OnRun()
	{
		telemetry.addData("Motor Encoder", motor.getCurrentPosition());
		telemetry.update();
	}
}

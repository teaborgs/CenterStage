package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.BaseOpMode;

@TeleOp(name = "Value Test", group = "Testing")
public class ValueTest extends BaseOpMode
{
	private DcMotorEx tumblerMotor;

	@Override
	protected void OnInitialize()
	{
		tumblerMotor = hardwareMap.get(DcMotorEx.class, "slot7");
		tumblerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		tumblerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		tumblerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

	@Override
	protected void OnRun()
	{
		telemetry.addData("Tumbler Encoder", tumblerMotor.getCurrentPosition());
		telemetry.update();
	}
}

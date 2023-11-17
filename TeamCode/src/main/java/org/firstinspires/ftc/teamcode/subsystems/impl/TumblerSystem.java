package org.firstinspires.ftc.teamcode.subsystems.impl;

import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;
import static org.firstinspires.ftc.teamcode.Utilities.CutPower;
import static org.firstinspires.ftc.teamcode.Utilities.RestorePower;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.SystemEx;

public final class TumblerSystem extends SystemEx
{
	private final DcMotorEx motor;

	public TumblerSystem(DcMotorEx motor)
	{
		this.motor = motor;
	}

	@Override
	public void Init()
	{
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motor.setDirection(DcMotorEx.Direction.REVERSE);
		motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		motor.setTargetPosition(Constants.Data.Tumbler.IDLE);
		motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
	}

	@Override
	public void Disable()
	{
		CutPower(motor);
	}

	@Override
	public void Enable()
	{
		RestorePower(motor);
	}

	@Override
	public Action MoveToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection)
	{
		return new SequentialAction(
				new SleepAction(delayDirection == Utilities.DelayDirection.BEFORE ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0),
				telemetryPacket -> {
					motor.setTargetPosition((int) position);
					motor.setPower(1);
					return false;
				},
				telemetryPacket -> Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > TOLERANCE,
				telemetryPacket -> {
					motor.setPower(0.05);
					return false;
				},
				new SleepAction(delayDirection == Utilities.DelayDirection.AFTER ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0)
		);
	}
}

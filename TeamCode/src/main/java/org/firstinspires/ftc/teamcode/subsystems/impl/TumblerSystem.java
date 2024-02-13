package org.firstinspires.ftc.teamcode.subsystems.impl;

import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;
import static org.firstinspires.ftc.teamcode.Utilities.CutPower;
import static org.firstinspires.ftc.teamcode.Utilities.RestorePower;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
	public void Setup()
	{
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setDirection(DcMotorEx.Direction.REVERSE);
		motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

	@Override
	public void Init()
	{
		this.Setup();
		motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motor.setTargetPosition(Constants.getTumblerIdle());
		motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		internal_Initialized = true;
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
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		return new SequentialAction(
				new SleepAction(delayDirection == Utilities.DelayDirection.BEFORE ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0),
				new InstantAction(() -> {
					motor.setTargetPosition((int) position);
					motor.setPower(1);
				}),
				telemetryPacket -> Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > TOLERANCE,
				new InstantAction(() -> motor.setPower(0.05)),
				new SleepAction(delayDirection == Utilities.DelayDirection.AFTER ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0)
		);
	}

	public void SetTargetPosition(double position, double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		Utilities.setTimeout(() -> {
			motor.setTargetPosition((int) position);
			motor.setPower(1);
		}, (int) (delay * 1000));
	}

	public void SetPower(double power, double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		Utilities.setTimeout(() -> motor.setPower(power), (int) (delay * 1000));
	}

	public void SetTargetPosition(double position)
	{
		SetTargetPosition(position, 0);
	}

	public void SetPower(double power)
	{
		SetPower(power, 0);
	}

	public void SetMode(DcMotorEx.RunMode mode)
	{
		motor.setMode(mode);
	}

	public double GetCurrentPosition()
	{
		return motor.getCurrentPosition();
	}

	public double GetTargetPosition()
	{
		return motor.getTargetPosition();
	}

	public double GetPower()
	{
		return motor.getPower();
	}

	public double GetCurrent(CurrentUnit unit)
	{
		return motor.getCurrent(unit);
	}
}
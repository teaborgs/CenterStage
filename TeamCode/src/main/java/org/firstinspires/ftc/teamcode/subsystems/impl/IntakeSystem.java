package org.firstinspires.ftc.teamcode.subsystems.impl;

import static org.firstinspires.ftc.teamcode.Utilities.CutPower;
import static org.firstinspires.ftc.teamcode.Utilities.RestorePower;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.SystemEx;

public final class IntakeSystem extends SystemEx
{
	private final DcMotorEx motor;
	private final Servo servo;

	public IntakeSystem(DcMotorEx motor, Servo servo)
	{
		this.motor = motor;
		this.servo = servo;
	}

	@Override
	public void Setup()
	{
		motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		motor.setDirection(DcMotorEx.Direction.REVERSE);
	}

	@Override
	public void Init()
	{
		this.Setup();
		servo.setPosition(Constants.getAntennaIdle());
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
		throw new UnsupportedOperationException();
	}

	public Action RunIntakeFor(double time, boolean reverse)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		return new SequentialAction(
				new InstantAction(() -> motor.setPower(Constants.getIntakeMaxPower() * (reverse ? -1 : 1))),
				new SleepAction(time),
				new InstantAction(() -> motor.setPower(0))
		);
	}

	public Action RunIntakeFor(double time)
	{
		return RunIntakeFor(time, false);
	}

	public Action RunIntakeWithAntennaFor(double time)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		return new SequentialAction(
				new InstantAction(() -> {
					servo.setPosition(Constants.getAntennaGrab());
					motor.setPower(Constants.getIntakeMaxPower());
				}),
				new SleepAction(time),
				new InstantAction(() -> {
					motor.setPower(0);
					servo.setPosition(Constants.getAntennaIdle());
				})
		);
	}

	public Action MoveAntennaToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		return new SequentialAction(
				new SleepAction(delayDirection == Utilities.DelayDirection.BEFORE ? delay : 0),
				new InstantAction(() -> servo.setPosition(position)),
				new SleepAction(delayDirection == Utilities.DelayDirection.AFTER ? delay : 0)
		);
	}

	public Action MoveAntennaToPositionWithDelay(double position, double delay)
	{
		return MoveAntennaToPositionWithDelay(position, delay, Utilities.DelayDirection.BEFORE);
	}

	public Action MoveAntennaToPosition(double position)
	{
		return MoveAntennaToPositionWithDelay(position, 0);
	}

	public void SetAntennaPosition(double position, double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		Utilities.setTimeout(() -> servo.setPosition(position), (int) (delay * 1000));
	}

	public void SetIntakePower(double power, double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		Utilities.setTimeout(() -> motor.setPower(power), (int) (delay * 1000));
	}

	public void SetAntennaPosition(double position)
	{
		SetAntennaPosition(position, 0);
	}

	public void SetIntakePower(double power)
	{
		SetIntakePower(power, 0);
	}

	public double GetCurrentPosition()
	{
		return motor.getCurrentPosition();
	}

	public double GetAntennaPosition()
	{
		return servo.getPosition();
	}
}
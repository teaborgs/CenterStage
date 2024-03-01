package org.firstinspires.ftc.teamcode.subsystems.impl;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.SystemEx;

import kotlin.NotImplementedError;

public final class TumblerSystem extends SystemEx
{
	private final Servo servo;

	public TumblerSystem(Servo s)
	{
		this.servo = s;
	}

	@Override
	public void Setup()
	{
	}

	@Override
	public void Init()
	{
		this.Setup();
		internal_Initialized = true;
	}

	@Override
	public void Disable()
	{
		servo.getController().pwmDisable();
	}

	@Override
	public void Enable()
	{
		servo.getController().pwmEnable();
	}

	@Override
	public Action MoveToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		return new SequentialAction(
				new SleepAction(delayDirection == Utilities.DelayDirection.BEFORE ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0),
				new InstantAction(() -> servo.setPosition(position)),
				new SleepAction(delayDirection == Utilities.DelayDirection.AFTER ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0)
		);
	}

	public void SetPosition(double position, double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		Utilities.setTimeout(() -> servo.setPosition(position), (int) (delay * 1000));
	}

	public void SetPosition(double position)
	{
		SetPosition(position, 0);
	}

	public double GetCurrentPosition()
	{
		// TODO: read axon 4th pin
		throw new NotImplementedError();
	}

	public double GetTargetPosition()
	{
		return servo.getPosition();
	}
}
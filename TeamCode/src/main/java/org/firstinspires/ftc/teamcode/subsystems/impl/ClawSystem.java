package org.firstinspires.ftc.teamcode.subsystems.impl;

import static org.firstinspires.ftc.teamcode.Utilities.CutPower;
import static org.firstinspires.ftc.teamcode.Utilities.RestorePower;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.SystemEx;


public final class ClawSystem extends SystemEx
{
	private final Servo servo;

	public ClawSystem(Servo servo)
	{
		this.servo = servo;
	}

	@Override
	public void Setup()
	{
	}

	@Override
	public void Init()
	{
		this.Setup();
		servo.setPosition(Constants.getClawIdle());
		internal_Initialized = true;
	}

	@Override
	public void Disable()
	{
		CutPower(servo);
	}

	@Override
	public void Enable()
	{
		RestorePower(servo);
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
		return servo.getPosition();
	}
}
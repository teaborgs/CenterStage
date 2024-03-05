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
	private final Servo servo1, servo2;

	public ClawSystem(Servo servo1, Servo servo2)
	{
		this.servo1 = servo1;
		this.servo2 = servo2;
	}

	@Override
	public void Setup()
	{
	}

	@Override
	public void Init()
	{
		this.Setup();
		servo1.setPosition(Constants.getClawIdle());
		servo2.setPosition(Constants.getClawIdle());
		internal_Initialized = true;
	}

	@Override
	public void Disable()
	{
		CutPower(servo1, servo2);
	}

	@Override
	public void Enable()
	{
		RestorePower(servo1, servo2);
	}

	@Override
	public Action MoveToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		return new SequentialAction(
				new SleepAction(delayDirection == Utilities.DelayDirection.BEFORE ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0),
				new InstantAction(() -> servo1.setPosition(position)),
				new InstantAction(() -> servo2.setPosition(position)),
				new SleepAction(delayDirection == Utilities.DelayDirection.AFTER ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0)
		);
	}

	public Action MoveFirstClawToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		return new SequentialAction(
				new SleepAction(delayDirection == Utilities.DelayDirection.BEFORE ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0),
				new InstantAction(() -> servo1.setPosition(position)),
				new SleepAction(delayDirection == Utilities.DelayDirection.AFTER ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0)
		);
	}

	public Action MoveSecondClawToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		return new SequentialAction(
				new SleepAction(delayDirection == Utilities.DelayDirection.BEFORE ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0),
				new InstantAction(() -> servo2.setPosition(position)),
				new SleepAction(delayDirection == Utilities.DelayDirection.AFTER ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0)
		);
	}

	public Action MoveFirstClawToPositionWithDelay(double position, double delay)
	{
		return MoveFirstClawToPositionWithDelay(position, delay, Utilities.DelayDirection.BOTH);
	}

	public Action MoveSecondClawToPositionWithDelay(double position, double delay)
	{
		return MoveSecondClawToPositionWithDelay(position, delay, Utilities.DelayDirection.BOTH);
	}

	public Action MoveFirstClawToPosition(double position)
	{
		return MoveFirstClawToPositionWithDelay(position, 0);
	}

	public Action MoveSecondClawToPosition(double position)
	{
		return MoveSecondClawToPositionWithDelay(position, 0);
	}

	/**
	 * Locks both servos
	 */
	public void CloseClaws()
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		servo1.setPosition(Constants.getClawBusy());
		servo2.setPosition(Constants.getClawBusy());
	}

	/**
	 * Unlocks both servos
	 */
	public void OpenClaws()
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		servo1.setPosition(Constants.getClawIdle());
		servo2.setPosition(Constants.getClawIdle());
	}

	public void OpenFirstClawWithDelay(double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		Utilities.setTimeout(() -> servo1.setPosition(Constants.getClawIdle()), (int) (delay * 1000));
	}

	public void OpenSecondClawWithDelay(double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		Utilities.setTimeout(() -> servo2.setPosition(Constants.getClawIdle()), (int) (delay * 1000));
	}

	public void OpenFirstClaw()
	{
		OpenFirstClawWithDelay(0);
	}

	public void OpenSecondClaw()
	{
		OpenSecondClawWithDelay(0);
	}

	public void CloseFirstClawWithDelay(double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		Utilities.setTimeout(() -> servo1.setPosition(Constants.getClawBusy()), (int) (delay * 1000));
	}

	public void CloseSecondClawWithDelay(double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		Utilities.setTimeout(() -> servo2.setPosition(Constants.getClawBusy()), (int) (delay * 1000));
	}

	public void CloseFirstClaw()
	{
		CloseFirstClawWithDelay(0);
	}

	public void CloseSecondClaw()
	{
		CloseSecondClawWithDelay(0);
	}

	public double GetFirstClawPosition()
	{
		return servo1.getPosition();
	}

	public double GetSecondClawPosition()
	{
		return servo2.getPosition();
	}
}
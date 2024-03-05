package org.firstinspires.ftc.teamcode.subsystems.impl;

import static org.firstinspires.ftc.teamcode.Utilities.CutPower;
import static org.firstinspires.ftc.teamcode.Utilities.RestorePower;
import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.SystemEx;

public final class DroneSystem extends SystemEx
{
	private final Servo leveler, shooter;

	public DroneSystem(Servo leveler, Servo shooter)
	{
		this.leveler = leveler;
		this.shooter = shooter;
	}

	@Override
	public void Setup()
	{
	}

	@Override
	public void Init()
	{
		this.Setup();
		leveler.setPosition(Constants.getPlaneLevelerIdle());
		shooter.setPosition(Constants.getPlaneShooterIdle());
		internal_Initialized = true;
	}

	@Override
	public void Disable()
	{
		CutPower(leveler, shooter);
	}

	@Override
	public void Enable()
	{
		RestorePower(leveler, shooter);
	}

	@Override
	public Action MoveToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection)
	{
		throw new UnsupportedOperationException();
	}

	public void LaunchDrone(double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		setTimeout(() -> {
			leveler.setPosition(Constants.getPlaneLevelerBusy());
			setTimeout(() -> {
				shooter.setPosition(Constants.getPlaneShooterBusy());
				setTimeout(() -> {
					leveler.setPosition(Constants.getPlaneLevelerIdle());
					shooter.setPosition(Constants.getPlaneShooterIdle());
				}, 200);
			}, 300);
		}, (int) (delay * 1000));
	}

	public void LaunchDrone()
	{
		LaunchDrone(0);
	}

	public double GetLevelerPosition()
	{
		return leveler.getPosition();
	}

	public double GetShooterPosition()
	{
		return shooter.getPosition();
	}
}
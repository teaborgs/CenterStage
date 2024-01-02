package org.firstinspires.ftc.teamcode.subsystems.impl;

import static org.firstinspires.ftc.teamcode.Utilities.CutPower;
import static org.firstinspires.ftc.teamcode.Utilities.RestorePower;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.SystemEx;

public final class IntakeSystem extends SystemEx
{
	private final PhotonDcMotor motor;

	public IntakeSystem(PhotonDcMotor motor) { this.motor = motor; }

	@Override
	public void Init() { motor.setMode(PhotonDcMotor.RunMode.RUN_WITHOUT_ENCODER); }

	@Override
	public void Disable() { CutPower(motor); }

	@Override
	public void Enable() { RestorePower(motor); }

	@Override
	public Action MoveToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection)
	{
		throw new UnsupportedOperationException();
	}

	public Action RunIntakeFor(double time)
	{
		return new SequentialAction(
				new InstantAction(() -> motor.setPower(Constants.getIntakeMaxPower())),
				new SleepAction(time),
				new InstantAction(() -> motor.setPower(0))
		);
	}
}
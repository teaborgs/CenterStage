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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.SystemEx;

public final class LiftSystem extends SystemEx
{
	private final DcMotorEx motor1, motor2;

	public LiftSystem(DcMotorEx motor1, DcMotorEx motor2)
	{
		this.motor1 = motor1;
		this.motor2 = motor2;
	}

	@Override
	public void Setup()
	{
		motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor1.setDirection(DcMotorEx.Direction.REVERSE);
		motor2.setDirection(DcMotorEx.Direction.REVERSE);
		motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

	@Override
	public void Init()
	{
		this.Setup();
		motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		motor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		motor1.setTargetPosition(Constants.getLiftLevels()[0]);
		motor2.setTargetPosition(Constants.getLiftLevels()[0]);
		internal_Initialized = true;
	}

	@Override
	public void Disable()
	{
		CutPower(motor1, motor2);
	}

	@Override
	public void Enable()
	{
		RestorePower(motor1, motor2);
	}

	@Override
	public Action MoveToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		return new SequentialAction(
				new SleepAction(delayDirection == Utilities.DelayDirection.BEFORE ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0),
				new InstantAction(() -> {
					motor1.setTargetPosition((int) position);
					motor2.setTargetPosition((int) position);
					motor1.setPower(1);
					motor2.setPower(1);
				}),
				telemetryPacket -> Math.abs(motor1.getCurrentPosition() - motor1.getTargetPosition()) > Constants.TOLERANCE || Math.abs(motor2.getCurrentPosition() - motor2.getTargetPosition()) > Constants.TOLERANCE,
				new InstantAction(() -> {
					motor1.setPower(0.05);
					motor2.setPower(0.05);
				}),
				new SleepAction(delayDirection == Utilities.DelayDirection.AFTER ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0)
		);
	}

	public void SetTargetPosition(double position, double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		Utilities.setTimeout(() -> {
			motor1.setTargetPosition((int) position);
			motor2.setTargetPosition((int) position);
		}, (int) (delay * 1000));
	}

	public void SetPower(double power, double delay)
	{
		if (!internal_Enabled || !internal_Initialized)
			throw new IllegalStateException("System is disabled or not initialized");
		Utilities.setTimeout(() -> {
			motor1.setPower(power);
			motor2.setPower(power);
		}, (int) (delay * 1000));
	}

	public void SetTargetPosition(double position)
	{
		SetTargetPosition(position, 0);
	}

	public void SetPower(double power)
	{
		SetPower(power, 0);
	}

	public double GetCurrentPosition()
	{
		return (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2.0;
	}

	public DcMotorEx getMotor1() { return motor1; }
	public DcMotorEx getMotor2() { return motor2; }

	public double GetPower()
	{
		return (motor1.getPower() + motor2.getPower()) / 2.0;
	}

	public double GetCurrent(CurrentUnit currentUnit)
	{
		return (motor1.getCurrent(currentUnit) + motor2.getCurrent(currentUnit)) / 2.0;
	}

	public double GetTargetPosition()
	{
		return (motor1.getTargetPosition() + motor2.getTargetPosition()) / 2.0;
	}

	public void SetMode(DcMotor.RunMode mode) { motor1.setMode(mode); motor2.setMode(mode); }

	public void SetZeroPowerBehaviour(DcMotorEx.ZeroPowerBehavior behavior) { motor1.setZeroPowerBehavior(behavior); motor2.setZeroPowerBehavior(behavior); }
}
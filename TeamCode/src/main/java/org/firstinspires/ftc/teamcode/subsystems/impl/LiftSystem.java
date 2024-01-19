package org.firstinspires.ftc.teamcode.subsystems.impl;

import static org.firstinspires.ftc.teamcode.Utilities.CutPower;
import static org.firstinspires.ftc.teamcode.Utilities.RestorePower;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
	public void Init()
	{
		motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motor1.setTargetPosition(Constants.getLiftPickup());
		motor2.setTargetPosition(Constants.getLiftPickup());
		motor2.setDirection(DcMotorSimple.Direction.REVERSE);
		motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		motor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
	}

	@Override
	public void Disable() { CutPower(motor1, motor2); }

	@Override
	public void Enable() { RestorePower(motor1, motor2); }

	@Override
	public Action MoveToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection)
	{
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
}
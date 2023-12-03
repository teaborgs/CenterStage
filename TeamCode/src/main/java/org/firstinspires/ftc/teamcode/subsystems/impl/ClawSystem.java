package org.firstinspires.ftc.teamcode.subsystems.impl;

import static org.firstinspires.ftc.teamcode.Utilities.CutPower;
import static org.firstinspires.ftc.teamcode.Utilities.RestorePower;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SystemEx;


public final class ClawSystem extends SystemEx
{
	private final Servo servo;

	public ClawSystem(Servo servo)
	{
		this.servo = servo;
	}

	@Override
	public void Init()
	{
		servo.setPosition(Constants.getClawIdle());
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
		return new SequentialAction(
				new SleepAction(delayDirection == Utilities.DelayDirection.BEFORE ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0),
				telemetryPacket -> {
					servo.setPosition(position);
					return false;
				},
				new SleepAction(delayDirection == Utilities.DelayDirection.AFTER ? delay : delayDirection == Utilities.DelayDirection.BOTH ? delay : 0)
		);
	}

	public Action ReleaseAtRest(MecanumDrive mecanumDrive)
	{
		return telemetryPacket -> {
			while (mecanumDrive.updatePoseEstimate().linearVel.x != 0 || mecanumDrive.updatePoseEstimate().linearVel.y != 0);
			servo.setPosition(Constants.getClawIdle());
			return false;
		};
	}
}
package org.firstinspires.ftc.teamcode.localizer.impl;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.localizer.Localizer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class TankDriveLocalizer implements Localizer
{
	public final List<Encoder> leftEncs, rightEncs;

	private double lastLeftPos, lastRightPos;

	private double inPerTick = 1.0;
	private final TankKinematics kinematics;

	public TankDriveLocalizer(TankDrive tankDrive, double inPerTick)
	{
		this.inPerTick = inPerTick;
		this.kinematics = tankDrive.kinematics;
		{
			List<Encoder> leftEncs = new ArrayList<>();
			for (DcMotorEx m : tankDrive.leftMotors) {
				Encoder e = new OverflowEncoder(new RawEncoder(m));
				leftEncs.add(e);
				lastLeftPos += e.getPositionAndVelocity().position;
			}
			lastLeftPos /= leftEncs.size();
			this.leftEncs = Collections.unmodifiableList(leftEncs);
		}

		{
			List<Encoder> rightEncs = new ArrayList<>();
			for (DcMotorEx m : tankDrive.rightMotors) {
				Encoder e = new OverflowEncoder(new RawEncoder(m));
				rightEncs.add(e);
				lastRightPos += e.getPositionAndVelocity().position;
			}
			lastRightPos /= rightEncs.size();
			this.rightEncs = Collections.unmodifiableList(rightEncs);
		}
	}

	@Override
	public Twist2dDual<Time> update()
	{
		double meanLeftPos = 0.0, meanLeftVel = 0.0;
		for (Encoder e : leftEncs) {
			PositionVelocityPair p = e.getPositionAndVelocity();
			meanLeftPos += p.position;
			meanLeftVel += p.velocity;
		}
		meanLeftPos /= leftEncs.size();
		meanLeftVel /= leftEncs.size();

		double meanRightPos = 0.0, meanRightVel = 0.0;
		for (Encoder e : rightEncs) {
			PositionVelocityPair p = e.getPositionAndVelocity();
			meanRightPos += p.position;
			meanRightVel += p.velocity;
		}
		meanRightPos /= rightEncs.size();
		meanRightVel /= rightEncs.size();

		TankKinematics.WheelIncrements<Time> twist = new TankKinematics.WheelIncrements<>(
				new DualNum<Time>(new double[]{
						meanLeftPos - lastLeftPos,
						meanLeftVel
				}).times(inPerTick),
				new DualNum<Time>(new double[]{
						meanRightPos - lastRightPos,
						meanRightVel,
				}).times(inPerTick)
		);

		lastLeftPos = meanLeftPos;
		lastRightPos = meanRightPos;

		return kinematics.forward(twist);
	}
}

package org.firstinspires.ftc.teamcode.localizer.impl;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.localizer.Localizer;

public class MecanumDriveLocalizer implements Localizer
{
	public final Encoder leftFront, leftRear, rightRear, rightFront;

	private int lastLeftFrontPos, lastLeftRearPos, lastRightRearPos, lastRightFrontPos;
	private Rotation2d lastHeading;

	private final IMU imu;
	private final MecanumKinematics kinematics;

	private final double inPerTick;

	public MecanumDriveLocalizer(MecanumDrive mecanumDrive, double inPerTick)
	{
		leftFront = new OverflowEncoder(new RawEncoder(mecanumDrive.leftFront));
		leftRear = new OverflowEncoder(new RawEncoder(mecanumDrive.leftBack));
		rightRear = new OverflowEncoder(new RawEncoder(mecanumDrive.rightBack));
		rightFront = new OverflowEncoder(new RawEncoder(mecanumDrive.rightFront));

		lastLeftFrontPos = leftFront.getPositionAndVelocity().position;
		lastLeftRearPos = leftRear.getPositionAndVelocity().position;
		lastRightRearPos = rightRear.getPositionAndVelocity().position;
		lastRightFrontPos = rightFront.getPositionAndVelocity().position;

		if(mecanumDrive.imu==null) throw new IllegalArgumentException("IMU cannot be null");
		if(mecanumDrive.kinematics==null) throw new IllegalArgumentException("Kinematics cannot be null");
		this.imu = mecanumDrive.imu.get();
		this.kinematics = mecanumDrive.kinematics;

		lastHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
		this.inPerTick = inPerTick;
	}

	@Override
	public Twist2dDual<Time> update()
	{
		PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
		PositionVelocityPair leftRearPosVel = leftRear.getPositionAndVelocity();
		PositionVelocityPair rightRearPosVel = rightRear.getPositionAndVelocity();
		PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

		Rotation2d heading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
		double headingDelta = heading.minus(lastHeading);

		Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
				new DualNum<Time>(new double[]{
						(leftFrontPosVel.position - lastLeftFrontPos),
						leftFrontPosVel.velocity,
				}).times(inPerTick),
				new DualNum<Time>(new double[]{
						(leftRearPosVel.position - lastLeftRearPos),
						leftRearPosVel.velocity,
				}).times(inPerTick),
				new DualNum<Time>(new double[]{
						(rightRearPosVel.position - lastRightRearPos),
						rightRearPosVel.velocity,
				}).times(inPerTick),
				new DualNum<Time>(new double[]{
						(rightFrontPosVel.position - lastRightFrontPos),
						rightFrontPosVel.velocity,
				}).times(inPerTick)
		));

		lastLeftFrontPos = leftFrontPosVel.position;
		lastLeftRearPos = leftRearPosVel.position;
		lastRightRearPos = rightRearPosVel.position;
		lastRightFrontPos = rightFrontPosVel.position;

		lastHeading = heading;

		return new Twist2dDual<>(
				twist.line,
				DualNum.cons(headingDelta, twist.angle.drop(1))
		);
	}
}

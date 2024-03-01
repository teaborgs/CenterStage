package org.firstinspires.ftc.teamcode.localizer.impl;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.localizer.Localizer;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer
{
	public static class Params
	{
		public double par0YTicks = -11877.854670649038; // y position of the first parallel encoder (in tick units)
		public double par1YTicks = 12064.825042508332; // y position of the second parallel encoder (in tick units)
		public double perpXTicks = 11671.267090997602; // x position of the perpendicular encoder (in tick units)
	}

	public static Params PARAMS = new Params();

	public final Encoder par0, par1, perp;
	public final IMU imu;

	public final double inPerTick;

	private int lastPar0Pos, lastPar1Pos, lastPerpPos;
	private Rotation2d lastHeading;
	private double lastRawHeadingVel, headingVelOffset;

	public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick)
	{
		par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
		par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "intake")));
		perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront")));
		this.imu = imu;

		par0.setDirection(DcMotorSimple.Direction.REVERSE);
		par1.setDirection(DcMotorSimple.Direction.REVERSE);

		lastPar0Pos = par0.getPositionAndVelocity().position;
		lastPar1Pos = par1.getPositionAndVelocity().position;
		lastPerpPos = perp.getPositionAndVelocity().position;
		if(imu!=null) lastHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

		this.inPerTick = inPerTick;

		FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
	}

	public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick)
	{
		this(hardwareMap, null, inPerTick);
	}

	// see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
	private double getHeadingVelocity()
	{
		double rawHeadingVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
		if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
			headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
		}
		lastRawHeadingVel = rawHeadingVel;
		return headingVelOffset + rawHeadingVel;
	}

	public Twist2dDual<Time> update()
	{
		PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
		PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
		PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
		Rotation2d headingRot = null;

		int par0PosDelta = par0PosVel.position - lastPar0Pos;
		int par1PosDelta = par1PosVel.position - lastPar1Pos;
		int perpPosDelta = perpPosVel.position - lastPerpPos;

		DualNum<Time> heading = new DualNum<>(new double[]{
				(par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
				(par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
		});

		if(imu!=null) {
			headingRot = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
			double headingDelta = headingRot.minus(lastHeading);
			double headingVel = getHeadingVelocity();
			heading = new DualNum<>(new double[]{
					headingDelta,
					headingVel,
			});
		}

		Twist2dDual<Time> twist = new Twist2dDual<>(
				new Vector2dDual<>(
						new DualNum<Time>(new double[]{
								(PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
								(PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
						}).times(inPerTick),
						new DualNum<Time>(new double[]{
								(PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
								(PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
						}).times(inPerTick)
				),
				heading
		);

		lastPar0Pos = par0PosVel.position;
		lastPar1Pos = par1PosVel.position;
		lastPerpPos = perpPosVel.position;
		if(imu!=null) lastHeading = headingRot;

		return twist;
	}
}
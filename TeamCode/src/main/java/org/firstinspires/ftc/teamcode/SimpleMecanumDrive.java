package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class SimpleMecanumDrive extends MecanumDrive
{
	private static final boolean RUN_USING_ENCODER = false;

	public SimpleMecanumDrive(HardwareMap hardwareMap, Pose2d pose, boolean useEncoders)
	{
		super(hardwareMap, pose);

		// broken
//		MotorConfigurationType motorType = MotorConfigurationType.getMotorType(DcMotorEx.class);
//		motorType.setAchieveableMaxRPMFraction(1.0);
//		setType(motorType);

		leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

		if (useEncoders) setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		else setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
	}

	public SimpleMecanumDrive(HardwareMap hardwareMap)
	{
		this(hardwareMap, new Pose2d(0, 0, 0), RUN_USING_ENCODER);
	}

	public SimpleMecanumDrive(HardwareMap hardwareMap, Pose2d pose)
	{
		this(hardwareMap, pose, RUN_USING_ENCODER);
	}

	public SimpleMecanumDrive(HardwareMap hardwareMap, boolean useEncoders)
	{
		this(hardwareMap, new Pose2d(0, 0, 0), useEncoders);
	}

	public void setMode(DcMotorEx.RunMode runMode)
	{
		leftFront.setMode(runMode);
		leftBack.setMode(runMode);
		rightFront.setMode(runMode);
		rightBack.setMode(runMode);
	}

	public void setType(MotorConfigurationType motorType)
	{
		leftFront.setMotorType(motorType);
		leftBack.setMotorType(motorType);
		rightFront.setMotorType(motorType);
		rightBack.setMotorType(motorType);
	}

	public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior)
	{
		leftFront.setZeroPowerBehavior(zeroPowerBehavior);
		leftBack.setZeroPowerBehavior(zeroPowerBehavior);
		rightFront.setZeroPowerBehavior(zeroPowerBehavior);
		rightBack.setZeroPowerBehavior(zeroPowerBehavior);
	}
}

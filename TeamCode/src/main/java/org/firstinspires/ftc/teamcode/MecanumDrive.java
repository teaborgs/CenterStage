package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.localizer.Localizer;
import org.firstinspires.ftc.teamcode.localizer.impl.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

/**
 * === Robot 1 Configuration ===
 * Control Hub:
 * > Servos:
 * slot0 - locker (locker)
 * slot1 - plane release (shooter)
 * slot2 - plane level (leveler)
 * > Motors:
 * slot0 - left-front wheel (leftFront) + par0 odometry
 * slot1 - left-back wheel (leftBack) + par1 odometry
 * slot2 - right-back wheel (rightBack) + perp odometry
 * slot3 - right-front wheel (rightFront)
 * Expansion Hub:
 * > Servos:
 * slot0 - rotator (rotator)
 * slot1 - claw (claw)
 * > Motors:
 * slot0 - intake (intake)
 * slot1 - lift1 (lift1)
 * slot2 - lift2 (lift2)
 * slot3 - tumbler (tumbler)
 */
@Config
public final class MecanumDrive
{
	public static class Params
	{
		public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
				RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
		public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
				RevHubOrientationOnRobot.UsbFacingDirection.UP;

		// drive model parameters
		public double inPerTick = 0;
		public double lateralInPerTick = 1;
		public double trackWidthTicks = 0;
		public double trackWidth = 0;

		// feedforward parameters (in tick units)
		public double kS = 0;
		public double kV = 0;
		public double kA = 0;

		// path profile parameters (in inches)
		public double maxWheelVel = 50;
		public double minProfileAccel = -30;
		public double maxProfileAccel = 50;

		// turn profile parameters (in radians)
		public double maxAngVel = Math.PI; // shared with path
		public double maxAngAccel = Math.PI;

		// path controller gains
		public double axialGain = 0.0;
		public double lateralGain = 0.0;
		public double headingGain = 0.0; // shared with turn

		public double axialVelGain = 0.0;
		public double lateralVelGain = 0.0;
		public double headingVelGain = 0.0; // shared with turn

		private boolean hasBeenInitialized = false;

		public void Init(Utilities.RobotType robotType)
		{
			if(hasBeenInitialized) return;
			hasBeenInitialized = true;
			if (Objects.requireNonNull(robotType) == Utilities.RobotType.ROBOT_1) {
				inPerTick = 5.2860537869735093113495079149827e-4; // 78.74 / 149535 148063 148225 = 78.74 / 148958
				lateralInPerTick = 0.0003726236031573551; // 0.0003485825272596426 0.00034130790197440175
				trackWidthTicks = 25563.724430353366; // 24432.28339319 24335.082134488
				trackWidth = 13.189;
				kS = 1.1133924997425; // 1.340230865032337
				kV =  0.00010747742980839806; // 0.00007465072936281095
				kA = 0.0000135;
				maxWheelVel = 70d;
				minProfileAccel = -30d;
				maxProfileAccel = 70d;
				axialGain = 6d;
				lateralGain = 10d;
				headingGain = 6d;
				axialVelGain = 0.25d;
				lateralVelGain = 0.25d;
				headingVelGain = 0.25d;
			} else if (robotType == Utilities.RobotType.ROBOT_2) {
				inPerTick = 0.0029438815568101; // 26786, 26758, 26697
				lateralInPerTick = -0.0018229134651507612; // DUBIOUS!!!1111!!!11!11!!1
				trackWidthTicks = -4587.149154167252;
				trackWidth = 13.6;
				kS = 1.45741682852298; //1.473189456272397, 1.488972102215801, 1.410088927080743
				kV = -0.0004179205649249333; //-0.0004183863917338, -0.000414885592430, -0.000420489710611
				kA = 0.00001;
				maxWheelVel = 50d;
				minProfileAccel = -20d;
				maxProfileAccel = 50d;
				axialGain = 6d;
				lateralGain = 6d;
				headingGain = 6d;
				axialVelGain = 0.5d;
				lateralVelGain = 1d;
				headingVelGain = 0.5d;
			}
		}
	}

	public static Params PARAMS = new Params();

	public final MecanumKinematics kinematics;
	public final TurnConstraints defaultTurnConstraints;
	public final VelConstraint defaultVelConstraint;
	public final AccelConstraint defaultAccelConstraint;

	public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

	public final VoltageSensor voltageSensor;

	public final IMU imu;

	public final Localizer localizer;
	public Pose2d pose;

	private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

	private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
	private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
	private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
	private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

	public MecanumDrive(HardwareMap hardwareMap, Pose2d pose)
	{
		PARAMS.Init(Globals.internal_GetRobotType(hardwareMap));
		this.pose = pose;

		this.kinematics = new MecanumKinematics(PARAMS.trackWidth, PARAMS.inPerTick / PARAMS.lateralInPerTick);
		this.defaultTurnConstraints = new TurnConstraints(PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
		this.defaultVelConstraint = new MinVelConstraint(Arrays.asList(
				this.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
				new AngularVelConstraint(PARAMS.maxAngVel)
		));
		this.defaultAccelConstraint = new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

		LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

		for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
			module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}

		leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
		leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
		rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
		rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

		leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
		leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

		leftFront.setPower(0);
		leftBack.setPower(0);
		rightBack.setPower(0);
		rightFront.setPower(0);

		imu = hardwareMap.get(IMU.class, "imu");
		IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
				PARAMS.logoFacingDirection,
				PARAMS.usbFacingDirection));
		imu.initialize(parameters);
		imu.resetYaw();

		voltageSensor = hardwareMap.voltageSensor.iterator().next();
		localizer = new ThreeDeadWheelLocalizer(hardwareMap, imu, PARAMS.inPerTick);
		FlightRecorder.write("MECANUM_PARAMS", PARAMS);
	}

	public MecanumDrive(HardwareMap hardwareMap)
	{
		this(hardwareMap, new Pose2d(0, 0, 0));
	}

	public void setDrivePowers(PoseVelocity2d powers)
	{
		MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(PARAMS.trackWidth).inverse(
				PoseVelocity2dDual.constant(powers, 1));

		double maxPowerMag = 1;
		for (DualNum<Time> power : wheelVels.all()) {
			maxPowerMag = Math.max(maxPowerMag, power.value());
		}

		leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
		leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
		rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
		rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
	}

	public final class FollowTrajectoryAction implements Action
	{
		public final TimeTrajectory timeTrajectory;
		private double beginTs = -1;

		private final double[] xPoints, yPoints;

		public FollowTrajectoryAction(TimeTrajectory t)
		{
			timeTrajectory = t;

			List<Double> disps = com.acmerobotics.roadrunner.Math.range(
					0, t.path.length(),
					Math.max(2, (int) Math.ceil(t.path.length() / 2)));
			xPoints = new double[disps.size()];
			yPoints = new double[disps.size()];
			for (int i = 0; i < disps.size(); i++) {
				Pose2d p = t.path.get(disps.get(i), 1).value();
				xPoints[i] = p.position.x;
				yPoints[i] = p.position.y;
			}
		}

		@Override
		public boolean run(@NonNull TelemetryPacket p)
		{
			double t;
			if (beginTs < 0) {
				beginTs = Actions.now();
				t = 0;
			} else {
				t = Actions.now() - beginTs;
			}

			Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
			targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

			PoseVelocity2d robotVelRobot = updatePoseEstimate();

			Pose2d error = txWorldTarget.value().minusExp(pose);

			boolean trajOver = t >= timeTrajectory.duration;
			boolean isPositionCorrect = error.position.norm() < 0.05;
			boolean isHeadingCorrect = Math.toDegrees(Math.abs(error.heading.toDouble())) < 0.05;
			boolean fallback = t - 0.25 >= timeTrajectory.duration;
			if ((trajOver && isPositionCorrect && isHeadingCorrect) || fallback) {
				leftFront.setPower(0);
				leftBack.setPower(0);
				rightBack.setPower(0);
				rightFront.setPower(0);

				return false;
			}

			PoseVelocity2dDual<Time> command = new HolonomicController(
					PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
					PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
			)
					.compute(txWorldTarget, pose, robotVelRobot);
			driveCommandWriter.write(new DriveCommandMessage(command));

			MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
			double voltage = voltageSensor.getVoltage();

			final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
			double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
			double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
			double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
			double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
			mecanumCommandWriter.write(new MecanumCommandMessage(
					voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
			));

			leftFront.setPower(leftFrontPower);
			leftBack.setPower(leftBackPower);
			rightBack.setPower(rightBackPower);
			rightFront.setPower(rightFrontPower);

			FlightRecorder.write("TARGET_POSE", new PoseMessage(txWorldTarget.value()));

			p.put("x", pose.position.x);
			p.put("y", pose.position.y);
			p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

			p.put("xError", error.position.x);
			p.put("yError", error.position.y);
			p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

			p.put("position norm error", error.position.norm());

			// only draw when active; only one drive action should be active at a time
			Canvas c = p.fieldOverlay();
			drawPoseHistory(c);

			c.setStroke("#4CAF50");
			drawRobot(c, txWorldTarget.value());

			c.setStroke("#3F51B5");
			drawRobot(c, pose);

			c.setStroke("#4CAF50FF");
			c.setStrokeWidth(1);
			c.strokePolyline(xPoints, yPoints);

			return true;
		}

		@Override
		public void preview(Canvas c)
		{
			c.setStroke("#4CAF507A");
			c.setStrokeWidth(1);
			c.strokePolyline(xPoints, yPoints);
		}
	}

	public final class TurnAction implements Action
	{
		private final TimeTurn turn;

		private double beginTs = -1;

		public TurnAction(TimeTurn turn)
		{
			this.turn = turn;
		}

		@Override
		public boolean run(@NonNull TelemetryPacket p)
		{
			double t;
			if (beginTs < 0) {
				beginTs = Actions.now();
				t = 0;
			} else {
				t = Actions.now() - beginTs;
			}

			if (t >= turn.duration) {
				leftFront.setPower(0);
				leftBack.setPower(0);
				rightBack.setPower(0);
				rightFront.setPower(0);

				return false;
			}

			Pose2dDual<Time> txWorldTarget = turn.get(t);
			targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

			PoseVelocity2d robotVelRobot = updatePoseEstimate();

			PoseVelocity2dDual<Time> command = new HolonomicController(
					PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
					PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
			)
					.compute(txWorldTarget, pose, robotVelRobot);
			driveCommandWriter.write(new DriveCommandMessage(command));

			MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
			double voltage = voltageSensor.getVoltage();
			final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
			double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
			double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
			double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
			double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
			mecanumCommandWriter.write(new MecanumCommandMessage(
					voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
			));

			leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
			leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
			rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
			rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

			Canvas c = p.fieldOverlay();
			drawPoseHistory(c);

			c.setStroke("#4CAF50");
			drawRobot(c, txWorldTarget.value());

			c.setStroke("#3F51B5");
			drawRobot(c, pose);

			c.setStroke("#7C4DFFFF");
			c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

			return true;
		}

		@Override
		public void preview(Canvas c)
		{
			c.setStroke("#7C4DFF7A");
			c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
		}
	}

	public PoseVelocity2d updatePoseEstimate()
	{
		Twist2dDual<Time> twist = localizer.update();
		pose = pose.plus(twist.value());

		poseHistory.add(pose);
		while (poseHistory.size() > 100) {
			poseHistory.removeFirst();
		}

		estimatedPoseWriter.write(new PoseMessage(pose));

		return twist.velocity().value();
	}

	private void drawPoseHistory(Canvas c)
	{
		double[] xPoints = new double[poseHistory.size()];
		double[] yPoints = new double[poseHistory.size()];

		int i = 0;
		for (Pose2d t : poseHistory) {
			xPoints[i] = t.position.x;
			yPoints[i] = t.position.y;

			i++;
		}

		c.setStrokeWidth(1);
		c.setStroke("#3F51B5");
		c.strokePolyline(xPoints, yPoints);
	}

	private static void drawRobot(Canvas c, Pose2d t)
	{
		final double ROBOT_RADIUS = 9;

		c.setStrokeWidth(1);
		c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

		Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
		Vector2d p1 = t.position.plus(halfv);
		Vector2d p2 = p1.plus(halfv);
		c.strokeLine(p1.x, p1.y, p2.x, p2.y);
	}

	public TrajectoryActionBuilder actionBuilder(Pose2d beginPose)
	{
		return new TrajectoryActionBuilder(
				TurnAction::new,
				FollowTrajectoryAction::new,
				beginPose, 1e-6, 0.0,
				defaultTurnConstraints,
				defaultVelConstraint, defaultAccelConstraint,
				0.25, 0.1
		);
	}
}
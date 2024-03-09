package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.ApproachWithDistSensor;
import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
import static org.firstinspires.ftc.teamcode.Utilities.WaitForMovementStop;
import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.opencv.TeamPropDetectionPipeline;

@Autonomous(name = "Auto Boss 🦅🦅🦅", group = "Auto")
public final class AutoBoss extends BaseOpMode
{
	private RobotHardware robotHardware;

	private boolean distanceSensorFault = false;

	private TeamPropDetectionPipeline detectionPipeline;

	private Utilities.Alliance currentAlliance = null;
	private Utilities.PathType currentPath = null;
	private Utilities.DetectionCase detectionCase = null;

	@Override
	protected void OnInitialize()
	{
		ConfigureAutonomous();
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2);
		Constants.Init();
		setAutonomous(true);

		robotHardware = new RobotHardware(hardwareMap);

		if (robotHardware.testDistanceSensor())
		{
			distanceSensorFault = true;
			telemetry.addLine("[ERROR] Distance sensor fault!");
			telemetry.update();
		}

		robotHardware.tumblerSystem.SetPosition(Constants.getTumblerLoad());

		// Camera and detection
		detectionPipeline = new TeamPropDetectionPipeline(currentAlliance, Globals.IsDebugging());
		robotHardware.setCameraPipeline(detectionPipeline);
		robotHardware.openCameraAsync();
	}

	@Override
	protected void OnRun()
	{
		robotHardware.closeCamera();
		if (currentPath == Utilities.PathType.SHORT) RunShort();
		else RunLong();
	}

	private void RunShort()
	{
		Pose2d purplePose = new Pose2d(0, 0, 0);
		Pose2d yellowPose = new Pose2d(0, 0, 0);
		Pose2d backdropIntermediaryPose = new Pose2d(0, 0, 0);
		Pose2d stackIntermediaryPose = new Pose2d(0, 0, 0);
		Pose2d stackPose = new Pose2d(0, 0, 0);
		Pose2d backdropPose = new Pose2d(0, 0, 0);
		Pose2d parkPose;
		double offset1 = -centimetersToInches(10);
		Action yellowApproach, backdropApproach, purpleApproach = null;

		if (currentAlliance == Utilities.Alliance.RED)
		{
			robotHardware.mecanumDrive.setStartPose(new Pose2d(0, centimetersToInches(8), 0));
			parkPose = new Pose2d(centimetersToInches(10), -centimetersToInches(80), -Math.PI / 2);
			if (detectionCase == Utilities.DetectionCase.CENTER)
			{ // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(92), -centimetersToInches(20), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(70), -centimetersToInches(96), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(60), -centimetersToInches(96), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(12), 0, -Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(12), centimetersToInches(110), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(72), centimetersToInches(182), -Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			} else if (detectionCase == Utilities.DetectionCase.LEFT)
			{ // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), centimetersToInches(8), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(85), -centimetersToInches(96), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(70), -centimetersToInches(96), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(12), 0, -Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(12), centimetersToInches(110), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(72), centimetersToInches(182), -Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.setTangent(purplePose.heading.toDouble())
						.lineToY(0)
						.setTangent(0)
						.lineToX(purplePose.position.x / 4 * 3)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			} else if (detectionCase == Utilities.DetectionCase.RIGHT)
			{ // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), -centimetersToInches(42), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(60), -centimetersToInches(96), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(75), -centimetersToInches(96), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(12), 0, -Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(12), centimetersToInches(110), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(72), centimetersToInches(182), -Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			}
		} else
		{
			offset1 *= -1;
			robotHardware.mecanumDrive.setStartPose(new Pose2d(centimetersToInches(4), centimetersToInches(4), 0));
			parkPose = new Pose2d(centimetersToInches(10), centimetersToInches(80), Math.PI / 2);
			if (detectionCase == Utilities.DetectionCase.CENTER)
			{ // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(94), centimetersToInches(20), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(65), centimetersToInches(96), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(52), centimetersToInches(96), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(13), 0, Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(13), -centimetersToInches(110), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(69), -centimetersToInches(182), Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			} else if (detectionCase == Utilities.DetectionCase.LEFT)
			{ // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), centimetersToInches(42), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(55), centimetersToInches(96), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(75), centimetersToInches(96), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(13), 0, Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(13), -centimetersToInches(110), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(69), -centimetersToInches(182), Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			} else if (detectionCase == Utilities.DetectionCase.RIGHT)
			{ // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), -centimetersToInches(8), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(85), centimetersToInches(97), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(55), centimetersToInches(97), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(14), 0, Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(14), -centimetersToInches(110), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(69), -centimetersToInches(182), Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.setTangent(purplePose.heading.toDouble())
						.lineToY(0)
						.setTangent(0)
						.lineToX(purplePose.position.x / 4 * 3)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			}
		}

		// Add fallbacks for distance sensor
		if (!distanceSensorFault)
		{
			yellowApproach = ApproachWithDistSensor(robotHardware, Constants.getBackdropDistance());
			backdropApproach = ApproachWithDistSensor(robotHardware, Constants.getBackdropDistance());
		} else
		{
			yellowApproach = robotHardware.mecanumDrive.actionBuilder(new Pose2d(yellowPose.position.x, yellowPose.position.y - offset1, yellowPose.heading.toDouble()))
					.setTangent(-yellowPose.heading.toDouble())
					.lineToYLinearHeading(yellowPose.position.y, yellowPose.component2())
					.build();
			backdropApproach = robotHardware.mecanumDrive.actionBuilder(new Pose2d(backdropPose.position.x, backdropPose.position.y - offset1, backdropPose.heading.toDouble()))
					.setTangent(backdropPose.heading.toDouble())
					.lineToYLinearHeading(backdropPose.position.y, backdropPose.heading.toDouble())
					.build();
		}

		Actions.runBlocking(RunSequentially(
				// Get claws to position
				robotHardware.clawSystem.MoveFirstClawToPosition(Constants.getClawBusy()),
				robotHardware.clawSystem.MoveSecondClawToPosition(Constants.getClawIdle()),

				WaitFor(0.5),

				// Place purple
				RunInParallel(
						purpleApproach,
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 1), // Move rotator to busy
						robotHardware.tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.75), // Move tumbler to backdrop
						robotHardware.intakeSystem.MoveAntennaToPositionWithDelay(Constants.getAntennaGrab() + 0.05d, 0.2), // Move antenna to grab
						RunSequentially(
								WaitFor(0.5),
								robotHardware.intakeSystem.RunIntakeFor(1, true)
						)
				),
				WaitForMovementStop(robotHardware),
				robotHardware.intakeSystem.MoveAntennaToPositionWithDelay(Constants.getAntennaIdle(), 0.25, Utilities.DelayDirection.AFTER),

				// Place yellow
				RunInParallel(
						RunSequentially(
								robotHardware.mecanumDrive.actionBuilder(purplePose)
										.splineToLinearHeading(new Pose2d(yellowPose.position.x, yellowPose.position.y - offset1, yellowPose.heading.toDouble()), yellowPose.heading.toDouble())
										.build(), // Get into position for yellow approach
								yellowApproach // Approach yellow
						),
						robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[1] + 100)
				),
				WaitForMovementStop(robotHardware),
				robotHardware.clawSystem.MoveFirstClawToPositionWithDelay(Constants.getClawIdle(), 0.25d, Utilities.DelayDirection.BOTH), // Open claw

				// Drive to stack
				RunInParallel(
						robotHardware.mecanumDrive.actionBuilder(yellowPose)
								.setTangent(-yellowPose.heading.toDouble())
								.splineToConstantHeading(backdropIntermediaryPose.component1(), -backdropIntermediaryPose.heading.toDouble())
								.lineToY(stackIntermediaryPose.position.y)
								.setTangent(-backdropIntermediaryPose.heading.toDouble())
								.splineToConstantHeading(new Vector2d(stackPose.position.x, stackPose.position.y - offset1 * -3), -stackPose.heading.toDouble(), (pose2dDual, posePath, v) -> 30, (pose2dDual, posePath, v) -> new MinMax(-30, 30))
								.setTangent(Math.PI / 2)
								.lineToY(stackPose.position.y, (pose2dDual, posePath, v) -> 20, (pose2dDual, posePath, v) -> new MinMax(-20, 20))
								.build(),
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.25),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
						robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[0])
				),

				// Take 2 pixels
				WaitForMovementStop(robotHardware),
				robotHardware.intakeSystem.RunIntakeWithAntennaFor(0.4),
				robotHardware.intakeSystem.RunIntakeFor(0.25),
				robotHardware.intakeSystem.RunIntakeWithAntennaFor(0.5),

				// Drive to intermediate backdrop position
				RunInParallel(
						robotHardware.mecanumDrive.actionBuilder(stackPose)
								.splineToConstantHeading(stackIntermediaryPose.component1(), stackIntermediaryPose.heading.toDouble(), (pose2dDual, posePath, v) -> 20, (pose2dDual, posePath, v) -> new MinMax(-20, 20))
								.lineToYConstantHeading(backdropIntermediaryPose.position.y)
								.splineToConstantHeading(new Vector2d(backdropPose.position.x, backdropPose.position.y - offset1), backdropPose.heading.toDouble())
								.build(),
						robotHardware.intakeSystem.RunIntakeFor(1.25),
						RunSequentially(
								robotHardware.tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 1.25),
								robotHardware.clawSystem.MoveFirstClawToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.BEFORE),
								robotHardware.clawSystem.MoveSecondClawToPositionWithDelay(Constants.getClawBusy(), 0.4, Utilities.DelayDirection.AFTER)
						)
				),

				// Drive to backdrop
				RunInParallel(
						backdropApproach,
						robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[3]),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.2)
				),
				WaitForMovementStop(robotHardware),

				// Drop stack pixels
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()), WaitFor(0.2),
				robotHardware.clawSystem.MoveFirstClawToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.AFTER),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
				WaitFor(0.5),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()), WaitFor(0.2),
				robotHardware.clawSystem.MoveSecondClawToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.AFTER),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
				WaitFor(0.2),

				// Reset positions and park
				RunInParallel(
						robotHardware.mecanumDrive.actionBuilder(backdropPose)
								.setTangent(-backdropPose.heading.toDouble())
								.splineToConstantHeading(parkPose.component1(), parkPose.heading.toDouble() * 2)
								.build(),
						robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[0]),
						robotHardware.rotatorSystem.MoveToPosition(Constants.getRotatorIdle())
				)
		));

		telemetry.clear();
		telemetry.addLine("Finished");
		telemetry.update();
	}

	public void RunLong()
	{
		Pose2d purplePose = new Pose2d(0, 0, 0);
		Pose2d firstStackPixel = new Pose2d(0, 0, 0);
		Pose2d backdropIntermediaryPose = new Pose2d(0, 0, 0);
		Pose2d yellowPose = new Pose2d(0, 0, 0);
		Pose2d stackPose = new Pose2d(0, 0, 0);
		Pose2d backdropPose = new Pose2d(0, 0, 0);
		double safeDistance = -centimetersToInches(30);
		Action yellowApproach, backdropApproach, initialStackApproach;
		if (currentAlliance == Utilities.Alliance.RED)
		{
			if (detectionCase == Utilities.DetectionCase.CENTER)
			{ // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(94), centimetersToInches(30), -Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(94), centimetersToInches(58), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(126), -centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(67), -centimetersToInches(218), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(126), centimetersToInches(58), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(75), -centimetersToInches(218), -Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.LEFT)
			{ // Left blue cas e is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(67), centimetersToInches(54), -Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(67), centimetersToInches(58), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(126), -centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(83), -centimetersToInches(218), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(126), centimetersToInches(58), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(65), -centimetersToInches(218), -Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.RIGHT)
			{ // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(67), 0, -Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(67), centimetersToInches(58), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(126), -centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(50), -centimetersToInches(218), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(126), centimetersToInches(58), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(75), -centimetersToInches(218), -Math.PI / 2);
			}
		} else
		{
			safeDistance *= -1;
			robotHardware.mecanumDrive.setStartPose(new Pose2d(centimetersToInches(4), centimetersToInches(12), 0));
			if (detectionCase == Utilities.DetectionCase.CENTER)
			{ // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(94), -centimetersToInches(30), Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(94), -centimetersToInches(58), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(124), centimetersToInches(140), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(70), centimetersToInches(223), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(124), -centimetersToInches(58), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(83), centimetersToInches(223), Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.LEFT)
			{ // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(67), 0, Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(67), -centimetersToInches(58), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(124), centimetersToInches(140), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(55), centimetersToInches(223), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(124), -centimetersToInches(58), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(83), centimetersToInches(223), Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.RIGHT)
			{ // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(67), -centimetersToInches(54), Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(67), -centimetersToInches(58), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(124), centimetersToInches(140), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(87), centimetersToInches(223), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(124), -centimetersToInches(58), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(65), centimetersToInches(223), Math.PI / 2);
			}
		}

		// Add fallbacks for distance sensor
		if (!distanceSensorFault)
		{
			yellowApproach = ApproachWithDistSensor(robotHardware, Constants.getBackdropDistance());
			backdropApproach = ApproachWithDistSensor(robotHardware, Constants.getBackdropDistance());
		} else
		{
			yellowApproach = robotHardware.mecanumDrive.actionBuilder(new Pose2d(yellowPose.position.x, yellowPose.position.y - safeDistance, yellowPose.heading.toDouble()))
					.setTangent(yellowPose.heading.toDouble())
					.lineToYLinearHeading(yellowPose.position.y, yellowPose.heading.toDouble())
					.build();
			backdropApproach = robotHardware.mecanumDrive.actionBuilder(new Pose2d(backdropPose.position.x, backdropPose.position.y - safeDistance, backdropPose.heading.toDouble()))
					.setTangent(backdropPose.heading.toDouble())
					.lineToYLinearHeading(backdropPose.position.y, backdropPose.heading.toDouble())
					.build();
		}

		if ((currentAlliance == Utilities.Alliance.RED && detectionCase != Utilities.DetectionCase.LEFT) ||
				(currentAlliance == Utilities.Alliance.BLUE && detectionCase != Utilities.DetectionCase.RIGHT))
		{
			initialStackApproach = RunInParallel(
					robotHardware.mecanumDrive.actionBuilder(purplePose)
							.setTangent(purplePose.heading.toDouble())
							.lineToY(firstStackPixel.position.y)
							.setTangent(0)
							.lineToX(firstStackPixel.position.x)
							.build(),
					robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
					robotHardware.rotatorSystem.MoveToPosition(Constants.getRotatorIdle())
			);
		} else initialStackApproach = new InstantAction(() -> {
		}); // No need to move to initial stack position

		Actions.runBlocking(RunSequentially(
				// Get claws to position
				robotHardware.clawSystem.MoveSecondClawToPosition(Constants.getClawIdle()),
				robotHardware.clawSystem.MoveFirstClawToPosition(Constants.getClawBusy()),
				WaitFor(0.75),
				// Place purple
				RunInParallel(
						robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
								.splineToLinearHeading(new Pose2d(purplePose.position.x, purplePose.position.y / 2, purplePose.heading.toDouble()), 0)
								.setTangent(purplePose.heading.toDouble())
								.lineToYConstantHeading(purplePose.position.y)
								.build(),
						robotHardware.tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.1),
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.3)
				),
				robotHardware.clawSystem.MoveFirstClawToPositionWithDelay(Constants.getClawIdle(), 0.25, Utilities.DelayDirection.BOTH),

				// Take one pixel from stack
				initialStackApproach,
				robotHardware.intakeSystem.RunIntakeWithAntennaFor(0.5),
				RunInParallel(
						robotHardware.intakeSystem.RunIntakeFor(0.75),
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.25),
						robotHardware.tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
						robotHardware.clawSystem.MoveFirstClawToPositionWithDelay(Constants.getClawBusy(), 1),
						robotHardware.clawSystem.MoveSecondClawToPositionWithDelay(Constants.getClawBusy(), 1)
				),
				WaitFor(0.75),

				// Drive to intermediate backdrop position
				robotHardware.mecanumDrive.actionBuilder(firstStackPixel)
						.setTangent(0)
						.splineToConstantHeading(new Vector2d(backdropIntermediaryPose.position.x, 0), backdropIntermediaryPose.heading.toDouble())
						.lineToYConstantHeading(backdropIntermediaryPose.position.y)
						.splineToConstantHeading(new Vector2d(yellowPose.position.x, yellowPose.position.y - safeDistance), yellowPose.heading.toDouble())
						.build(),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
				robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.25),
				WaitForMovementStop(robotHardware)
		));
		robotHardware.mecanumDrive.updatePoseEstimate();
		Actions.runBlocking(RunSequentially(
				// Drive to backdrop
				RunInParallel(
						RunSequentially(
								robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
										.turnTo(yellowPose.component2())
										.build(),
								yellowApproach
						),
						robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[3] / 5d * 3)
				),
				WaitForMovementStop(robotHardware),

				// Place yellow
				robotHardware.clawSystem.MoveFirstClawToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.AFTER),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle())
		));
		robotHardware.mecanumDrive.updatePoseEstimate();
		Actions.runBlocking(RunSequentially(
				// Place stack pixel
				RunInParallel(
						robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
								.setTangent(0)
								.lineToXConstantHeading(backdropPose.position.x)
								.build()
				),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
				robotHardware.clawSystem.MoveSecondClawToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.BOTH),

				// Drive to stack
				RunInParallel(
						robotHardware.mecanumDrive.actionBuilder(backdropPose)
								.setTangent(-backdropPose.heading.toDouble())
								.splineToConstantHeading(new Vector2d(backdropIntermediaryPose.position.x, backdropIntermediaryPose.position.y), -backdropPose.heading.toDouble())
								.lineToYConstantHeading(stackPose.position.y, (pose2dDual, posePath, v) -> 30)
								.build(),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
						robotHardware.liftSystem.MoveToPositionWithDelay(Constants.getLiftLevels()[0], 0.3)
				),

				// Take 2 pixels
				WaitForMovementStop(robotHardware),
				robotHardware.intakeSystem.RunIntakeWithAntennaFor(0.75),
				robotHardware.intakeSystem.RunIntakeFor(0.25),
				robotHardware.intakeSystem.RunIntakeWithAntennaFor(0.75),

				// Drive to intermediate backdrop position
				RunInParallel(
						robotHardware.mecanumDrive.actionBuilder(stackPose)
								.splineToConstantHeading(new Vector2d(backdropIntermediaryPose.position.x, 0), backdropIntermediaryPose.heading.toDouble())
								.lineToYConstantHeading(backdropIntermediaryPose.position.y)
								.splineToConstantHeading(new Vector2d(backdropPose.position.x, backdropPose.position.y - safeDistance), backdropPose.heading.toDouble())
								.build(),
						robotHardware.intakeSystem.RunIntakeFor(1.25),
						RunSequentially(
								robotHardware.tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 1.25),
								robotHardware.clawSystem.MoveFirstClawToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.BEFORE),
								robotHardware.clawSystem.MoveSecondClawToPositionWithDelay(Constants.getClawBusy(), 0.4, Utilities.DelayDirection.AFTER)
						)
				),

				// Drive to backdrop
				RunInParallel(
						backdropApproach,
						robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[3]),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.2)
				),
				WaitForMovementStop(robotHardware),

				// Drop stack pixels
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()), WaitFor(0.2),
				robotHardware.clawSystem.MoveFirstClawToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.AFTER),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
				WaitFor(0.5),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()), WaitFor(0.2),
				robotHardware.clawSystem.MoveSecondClawToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.AFTER),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
				WaitFor(0.2),

				// Reset positions
				RunInParallel(
						robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[0]),
						robotHardware.rotatorSystem.MoveToPosition(Constants.getRotatorIdle())
				)
		));

		telemetry.clear();
		telemetry.addLine("Finished");
		telemetry.update();
	}

	@Override
	protected void WhileWaitingForStart()
	{
		detectionCase = detectionPipeline.getDetectionCase();

		telemetry.addData("[INFO] Alliance", currentAlliance.name());
		telemetry.addData("[INFO] Path", currentPath.name());
		telemetry.addData("[INFO] Case", detectionCase.name());

		telemetry.addLine();
		telemetry.addLine("> We are ready to go!");
		telemetry.addLine("> Make sure to place robot ~4 fingers away from the left hand side of the start tile!");
		telemetry.addLine("> Press play to start...");

		telemetry.update();
	}

	private void ConfigureAutonomous()
	{
		telemetry.clear();
		telemetry.addLine("[CONFIGURE] Please select an alliance");
		telemetry.addLine("[CONFIGURE] Press B for Red Alliance");
		telemetry.addLine("[CONFIGURE] Press X for Blue Alliance");
		telemetry.update();
		while (currentAlliance == null && !isStopRequested())
		{
			if (gamepad1.b || gamepad2.b) currentAlliance = Utilities.Alliance.RED;
			else if (gamepad1.x || gamepad2.x) currentAlliance = Utilities.Alliance.BLUE;
		}

		telemetry.clear();
		telemetry.addLine("[CONFIGURE] Please select a path");
		telemetry.addLine("[CONFIGURE] Press A for Short Path");
		telemetry.addLine("[CONFIGURE] Press Y for Long Path");
		telemetry.update();
		while (currentPath == null && !isStopRequested())
		{
			if (gamepad1.a || gamepad2.a) currentPath = Utilities.PathType.SHORT;
			else if (gamepad1.y || gamepad2.y) currentPath = Utilities.PathType.LONG;
		}

		telemetry.clearAll();
		telemetry.update();
	}
}
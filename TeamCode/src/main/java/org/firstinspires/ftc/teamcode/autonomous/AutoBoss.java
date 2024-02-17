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
public class AutoBoss extends BaseOpMode
{
	private RobotHardware robotHardware;

	private boolean faultyDistanceSensor = false;

	private TeamPropDetectionPipeline detectionPipeline;

	private Utilities.Alliance currentAlliance = null;
	private Utilities.PathType currentPath = null;
	private Utilities.ParkingPosition parkingType = null;
	private Utilities.DetectionCase detectionCase = null;
	private double longDelay = -1;

	@Override
	protected void OnInitialize()
	{
		ConfigureAutonomous();
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2);
		Constants.Init();
		setAutonomous(true);

		robotHardware = new RobotHardware(hardwareMap);

		if (robotHardware.testDistanceSensor()) {
			faultyDistanceSensor = true;
			telemetry.addLine("[ERROR] Distance sensor is faulty!");
			telemetry.update();
		}

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

		if (currentAlliance == Utilities.Alliance.RED) {
			robotHardware.mecanumDrive.setStartPose(new Pose2d(0, centimetersToInches(8), 0));
			parkPose = new Pose2d(centimetersToInches(parkingType == Utilities.ParkingPosition.CENTER ? 130 : 10), -centimetersToInches(80), -Math.PI / 2);
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(92), -centimetersToInches(20), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(70), -centimetersToInches(96), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(60), -centimetersToInches(96), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(12), 0, -Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(12), centimetersToInches(110), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(70), centimetersToInches(180), -Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), centimetersToInches(10), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(85), -centimetersToInches(96), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(70), -centimetersToInches(96), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(12), 0, -Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(12), centimetersToInches(110), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(70), centimetersToInches(180), -Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.setTangent(purplePose.heading.toDouble())
						.lineToY(0)
						.setTangent(0)
						.lineToX(purplePose.position.x / 4 * 3)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), -centimetersToInches(42), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(60), -centimetersToInches(96), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(75), -centimetersToInches(96), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(12), 0, -Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(12), centimetersToInches(110), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(70), centimetersToInches(180), -Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			}
		} else {
			offset1 *= -1;
			robotHardware.mecanumDrive.setStartPose(new Pose2d(centimetersToInches(4), centimetersToInches(4), 0));
			parkPose = new Pose2d(centimetersToInches(parkingType == Utilities.ParkingPosition.CENTER ? 130 : 10), centimetersToInches(80), Math.PI / 2);
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(94), centimetersToInches(20), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(65), centimetersToInches(96), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(52), centimetersToInches(96), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(13), 0, Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(13), -centimetersToInches(110), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(67.5), -centimetersToInches(180), Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), centimetersToInches(42), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(55), centimetersToInches(96), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(75), centimetersToInches(96), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(13), 0, Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(13), -centimetersToInches(110), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(67.5), -centimetersToInches(180), Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), -centimetersToInches(9), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(85), centimetersToInches(97), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(55), centimetersToInches(97), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(14), 0, Math.PI / 2);
				stackIntermediaryPose = new Pose2d(centimetersToInches(14), -centimetersToInches(110), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(67.5), -centimetersToInches(180), Math.PI / 2);
				purpleApproach = robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
						.setTangent(purplePose.heading.toDouble())
						.lineToY(0)
						.setTangent(0)
						.lineToX(purplePose.position.x / 4 * 3)
						.splineToLinearHeading(purplePose, purplePose.heading.toDouble())
						.build();
			}
		}

		// Add fallbacks for faulty distance sensor
		if (!faultyDistanceSensor) {
			yellowApproach = ApproachWithDistSensor(robotHardware, Constants.getBackdropDistance());
			backdropApproach = ApproachWithDistSensor(robotHardware, Constants.getBackdropDistance());
		} else {
			yellowApproach = robotHardware.mecanumDrive.actionBuilder(new Pose2d(yellowPose.position.x, yellowPose.position.y - offset1, yellowPose.heading.toDouble()))
					.setTangent(-yellowPose.heading.toDouble())
					.lineToYLinearHeading(yellowPose.position.y, yellowPose.component2())
					.build();
			backdropApproach = robotHardware.mecanumDrive.actionBuilder(new Pose2d(backdropPose.position.x, backdropPose.position.y - offset1, backdropPose.heading.toDouble()))
					.setTangent(backdropPose.heading.toDouble())
					.lineToYLinearHeading(backdropPose.position.y, backdropPose.heading.toDouble())
					.build();
		}

		Actions.runBlocking(
				RunSequentially(
						robotHardware.clawSystem.MoveToPosition(Constants.getClawBusy()), // Close claw
						// Place purple
						RunInParallel(
								purpleApproach,
								robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75), // Move rotator to busy
								robotHardware.tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.5), // Move tumbler to backdrop
								robotHardware.intakeSystem.MoveAntennaToPositionWithDelay(Constants.getAntennaGrab(), 0.2), // Move antenna to idle
								RunSequentially(
										WaitFor(0.5),
										robotHardware.intakeSystem.RunIntakeFor(0.75, true)
								)
						),
						WaitForMovementStop(robotHardware), // Wait for movement to stop
						robotHardware.intakeSystem.MoveAntennaToPositionWithDelay(Constants.getAntennaIdle(), 0.25, Utilities.DelayDirection.AFTER),
						RunInParallel(
								RunSequentially(
										robotHardware.mecanumDrive.actionBuilder(purplePose)
												.splineToLinearHeading(new Pose2d(yellowPose.position.x, yellowPose.position.y - offset1, yellowPose.heading.toDouble()), yellowPose.heading.toDouble())
												.build(), // Get into position for yellow approach
										yellowApproach // Approach yellow
								),
								robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[1])
						),
						WaitForMovementStop(robotHardware),
						robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.25d, Utilities.DelayDirection.BOTH), // Open claw
						RunInParallel(
								robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.25),
								robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
								robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[0]),
								robotHardware.mecanumDrive.actionBuilder(yellowPose)
										.setTangent(-yellowPose.heading.toDouble())
										.splineToConstantHeading(backdropIntermediaryPose.component1(), -backdropIntermediaryPose.heading.toDouble())
										.lineToY(stackIntermediaryPose.position.y)
										.setTangent(-backdropIntermediaryPose.heading.toDouble())
										.splineToConstantHeading(new Vector2d(stackPose.position.x, stackPose.position.y - offset1 * -3), -stackPose.heading.toDouble(), (pose2dDual, posePath, v) -> 30, (pose2dDual, posePath, v) -> new MinMax(-30, 30))
										.setTangent(Math.PI / 2)
										.lineToY(stackPose.position.y, (pose2dDual, posePath, v) -> 20, (pose2dDual, posePath, v) -> new MinMax(-20, 20))
										.build()
						),
						// Take 2 pixels
						WaitForMovementStop(robotHardware),
						robotHardware.intakeSystem.RunIntakeWithAntennaFor(0.4),
						robotHardware.intakeSystem.RunIntakeFor(0.1),
						robotHardware.intakeSystem.RunIntakeWithAntennaFor(0.5),
						// Drive to intermediate backdrop position
						RunInParallel(
								robotHardware.mecanumDrive.actionBuilder(stackPose)
										.splineToConstantHeading(stackIntermediaryPose.component1(), stackIntermediaryPose.heading.toDouble(), (pose2dDual, posePath, v) -> 20, (pose2dDual, posePath, v) -> new MinMax(-20, 20))
										.lineToYConstantHeading(backdropIntermediaryPose.position.y)
										.splineToConstantHeading(new Vector2d(backdropPose.position.x, backdropPose.position.y - offset1), backdropPose.heading.toDouble())
										.build(),
								robotHardware.intakeSystem.RunIntakeFor(0.5),
								RunSequentially(
										robotHardware.tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
										robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.4, Utilities.DelayDirection.AFTER)
								)
						),
						// Drive to backdrop
						RunInParallel(
								backdropApproach,
								robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[2]),
								robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
								robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.2)
						),
						WaitForMovementStop(robotHardware),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
						// Drop first stack pixel
						robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.4, Utilities.DelayDirection.AFTER),
						RunInParallel(
								robotHardware.intakeSystem.RunIntakeFor(1),
								robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
								robotHardware.liftSystem.MoveToPositionWithDelay(Constants.getLiftSuspenderIdle(), 0.2),
								robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2)
						),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
						// Pickup second stack pixel and drop
						robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.4, Utilities.DelayDirection.AFTER),
						RunInParallel(
								robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[3]),
								robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
								robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.2)
						),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
						// Reset positions and park
						robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.AFTER),
						RunInParallel(
								robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								robotHardware.liftSystem.MoveToPositionWithDelay(Constants.getLiftSuspenderIdle(), 0.2),
								robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.1)
						),
						robotHardware.mecanumDrive.actionBuilder(backdropPose)
								.setTangent(-backdropPose.heading.toDouble())
								.splineToConstantHeading(parkPose.component1(), parkPose.heading.toDouble() * 2)
								.build()
				)
		);
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
		if (currentAlliance == Utilities.Alliance.RED) {
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(94), centimetersToInches(30), -Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(94), centimetersToInches(54), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(125), -centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(67), -centimetersToInches(223), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(125), centimetersToInches(56), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(83), -centimetersToInches(223), -Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue cas e is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(67), centimetersToInches(54), -Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(67), centimetersToInches(54), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(125), -centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(83), -centimetersToInches(223), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(125), centimetersToInches(56), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(65), -centimetersToInches(223), -Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(67), 0, -Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(67), centimetersToInches(54), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(125), -centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(56), -centimetersToInches(223), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(125), centimetersToInches(56), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(83), -centimetersToInches(223), -Math.PI / 2);
			}
		} else {
			safeDistance *= -1;
			robotHardware.mecanumDrive.setStartPose(new Pose2d(centimetersToInches(4), centimetersToInches(12), 0));
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(94), -centimetersToInches(30), Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(94), -centimetersToInches(54), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(123), centimetersToInches(140), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(70), centimetersToInches(223), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(123), -centimetersToInches(56), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(83), centimetersToInches(223), Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(67), 0, Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(67), -centimetersToInches(54), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(123), centimetersToInches(140), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(55), centimetersToInches(223), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(123), -centimetersToInches(56), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(83), centimetersToInches(223), Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(67), -centimetersToInches(54), Math.PI / 2);
				firstStackPixel = new Pose2d(centimetersToInches(67), -centimetersToInches(54), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(123), centimetersToInches(140), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(87), centimetersToInches(223), Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(123), -centimetersToInches(56), Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(65), centimetersToInches(223), Math.PI / 2);
			}
		}

		// Add fallbacks for faulty distance sensor
		if (!faultyDistanceSensor) {
			yellowApproach = ApproachWithDistSensor(robotHardware, Constants.getBackdropDistance());
			backdropApproach = ApproachWithDistSensor(robotHardware, Constants.getBackdropDistance());
		} else {
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
				(currentAlliance == Utilities.Alliance.BLUE && detectionCase != Utilities.DetectionCase.RIGHT)) {
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
				WaitFor(longDelay),
				robotHardware.clawSystem.MoveToPosition(Constants.getClawBusy()),
				// Place purple
				RunInParallel(
						robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
								.splineToLinearHeading(new Pose2d(purplePose.position.x, purplePose.position.y / 2, purplePose.heading.toDouble()), 0)
								.setTangent(purplePose.heading.toDouble())
								.lineToYConstantHeading(purplePose.position.y)
								.build(),
						robotHardware.tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.25),
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5)
				),
				robotHardware.clawSystem.MoveToPosition(Constants.getClawIdle()),
				initialStackApproach,
				robotHardware.intakeSystem.RunIntakeWithAntennaFor(0.5),
				// Drive to intermediate backdrop position
				RunInParallel(
						robotHardware.mecanumDrive.actionBuilder(firstStackPixel)
								.setTangent(0)
								.splineToConstantHeading(new Vector2d(backdropIntermediaryPose.position.x, 0), backdropIntermediaryPose.heading.toDouble())
								.lineToYConstantHeading(backdropIntermediaryPose.position.y)
								.splineToConstantHeading(new Vector2d(yellowPose.position.x, yellowPose.position.y - safeDistance), yellowPose.heading.toDouble())
								.build(),
						RunSequentially(
								RunSequentially(
										RunInParallel(
												robotHardware.intakeSystem.RunIntakeFor(1),
												robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.4),
												robotHardware.tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.6)
										),
										robotHardware.clawSystem.MoveToPosition(Constants.getClawBusy())
								),
								WaitFor(0.5),
								robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.2),
								robotHardware.tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.2)
						)
				),
				// Drive to backdrop
				WaitForMovementStop(robotHardware)
		));
		robotHardware.mecanumDrive.updatePoseEstimate();
		Actions.runBlocking(
				RunSequentially(
						RunInParallel(
								RunSequentially(
										robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
												.turnTo(yellowPose.component2())
												.build(),
										yellowApproach
								),
								robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[3] / 5d * 3)
						),
						// Drop pixel
						RunInParallel(
								RunSequentially(
										robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.BOTH),
										RunInParallel(
												robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
												robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
												robotHardware.liftSystem.MoveToPositionWithDelay(Constants.getLiftSuspenderIdle(), 0.2)
										)
								),
								robotHardware.intakeSystem.RunIntakeFor(1)
						),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
				)
		);
		robotHardware.mecanumDrive.updatePoseEstimate();
		Actions.runBlocking(RunSequentially(
				// Pickup stack pixel and drop
				RunInParallel(
						robotHardware.mecanumDrive.actionBuilder(robotHardware.mecanumDrive.pose)
								.setTangent(0)
								.lineToXConstantHeading(backdropPose.position.x)
								.build(),
						robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER)
				),
				RunInParallel(
						robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[2]),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
						robotHardware.rotatorSystem.MoveToPosition(Constants.getRotatorBusy())
				),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
				robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.BOTH),
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
				robotHardware.intakeSystem.RunIntakeWithAntennaFor(0.4),
				robotHardware.intakeSystem.RunIntakeFor(0.1),
				robotHardware.intakeSystem.RunIntakeWithAntennaFor(0.5),
				// Drive to intermediate backdrop position
				RunInParallel(
						robotHardware.mecanumDrive.actionBuilder(stackPose)
								.splineToConstantHeading(new Vector2d(backdropIntermediaryPose.position.x, 0), backdropIntermediaryPose.heading.toDouble())
								.lineToYConstantHeading(backdropIntermediaryPose.position.y)
								.splineToConstantHeading(new Vector2d(backdropPose.position.x, backdropPose.position.y - safeDistance), backdropPose.heading.toDouble())
								.build(),
						robotHardware.intakeSystem.RunIntakeFor(0.5),
						RunSequentially(
								robotHardware.tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
								robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.4, Utilities.DelayDirection.AFTER)
						)
				),
				// Drive to backdrop
				RunInParallel(
						backdropApproach,
						robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[2]),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.2)
				),
				WaitForMovementStop(robotHardware),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
				// Drop first stack pixel
				robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.4, Utilities.DelayDirection.AFTER),
				RunInParallel(
						robotHardware.intakeSystem.RunIntakeFor(1),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
						robotHardware.liftSystem.MoveToPositionWithDelay(Constants.getLiftSuspenderIdle(), 0.2),
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2)
				),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
				// Pickup second stack pixel and drop
				robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.4, Utilities.DelayDirection.AFTER),
				RunInParallel(
						robotHardware.liftSystem.MoveToPosition(Constants.getLiftLevels()[3]),
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.2)
				),
				robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
				// Reset positions and park
				robotHardware.clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.AFTER),
				RunInParallel(
						robotHardware.tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
						robotHardware.liftSystem.MoveToPositionWithDelay(Constants.getLiftSuspenderIdle(), 0.2),
						robotHardware.rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.1)
				),
				WaitFor(30)
		));
	}

	@Override
	protected void WhileWaitingForStart()
	{
		detectionCase = detectionPipeline.getDetectionCase();

		telemetry.addData("[INFO] Alliance", currentAlliance.name());
		telemetry.addData("[INFO] Path", currentPath.name());
		telemetry.addData("[INFO] Parking", parkingType.name());
		telemetry.addData("[INFO] Case", detectionCase.name());
		if (currentPath == Utilities.PathType.LONG) telemetry.addData("[INFO] Timeout", longDelay);

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
		telemetry.addLine("[CONFIGURE] Press A for Red Alliance");
		telemetry.addLine("[CONFIGURE] Press B for Blue Alliance");
		telemetry.update();
		while (currentAlliance == null && !isStopRequested()) {
			if (gamepad1.a || gamepad2.a) currentAlliance = Utilities.Alliance.RED;
			else if (gamepad1.b || gamepad2.b) currentAlliance = Utilities.Alliance.BLUE;
		}

		telemetry.clear();
		telemetry.addLine("[CONFIGURE] Please select a path");
		telemetry.addLine("[CONFIGURE] Press X for Short Path");
		telemetry.addLine("[CONFIGURE] Press Y for Long Path");
		telemetry.update();
		while (currentPath == null && !isStopRequested()) {
			if (gamepad1.x || gamepad2.x) currentPath = Utilities.PathType.SHORT;
			else if (gamepad1.y || gamepad2.y) currentPath = Utilities.PathType.LONG;
		}

		telemetry.clear();
		telemetry.addLine("[CONFIGURE] Please select a parking strategy");
		telemetry.addLine("[CONFIGURE] Press A for Center Parking");
		telemetry.addLine("[CONFIGURE] Press B for Wall Parking");
		telemetry.update();
		while (parkingType == null && !isStopRequested()) {
			if (gamepad1.a || gamepad2.a) parkingType = Utilities.ParkingPosition.CENTER;
			else if (gamepad1.b || gamepad2.b) parkingType = Utilities.ParkingPosition.WALL;
		}

		if (currentPath == Utilities.PathType.LONG) {
			telemetry.clear();
			telemetry.addLine("[CONFIGURE] Long path selected. Please select a delay");
			telemetry.addLine("[CONFIGURE] Press X for 0 second delay");
			telemetry.addLine("[CONFIGURE] Press Y for 2.5 second delay");
			telemetry.update();
			while (longDelay < 0 && !isStopRequested()) {
				if (gamepad1.x || gamepad2.x) longDelay = 0;
				else if (gamepad1.y || gamepad2.y) longDelay = 2.5;
			}
		}

		telemetry.clearAll();
		telemetry.update();
	}
}
package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.ApproachWithDistSensor;
import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
import static org.firstinspires.ftc.teamcode.Utilities.WaitForMovementStop;
import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.opencv.TeamPropDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.impl.ClawSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.LiftSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.RotatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.TumblerSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Boss", group = "Auto")
public class AutoBoss extends BaseOpMode
{
	private MecanumDrive mecanumDrive;
	private TumblerSystem tumblerSystem;
	private RotatorSystem rotatorSystem;
	private ClawSystem clawSystem;
	private IntakeSystem intakeSystem;
	private LiftSystem liftSystem;

	private Rev2mDistanceSensor distanceSensor;
	private boolean faultyDistanceSensor = false;

	private OpenCvCamera camera;
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

		// Systems
		mecanumDrive = new MecanumDrive(hardwareMap);

		tumblerSystem = new TumblerSystem(hardwareMap.get(DcMotorEx.class, "tumbler"));
		intakeSystem = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"), hardwareMap.get(Servo.class, "antenna"));
		rotatorSystem = new RotatorSystem(hardwareMap.get(Servo.class, "rotator"));
		clawSystem = new ClawSystem(hardwareMap.get(Servo.class, "claw"));
		liftSystem = new LiftSystem(hardwareMap.get(DcMotorEx.class, "lift1"), hardwareMap.get(DcMotorEx.class, "lift2"));

		// Initialize systems
		tumblerSystem.Init();
		intakeSystem.Init();
		rotatorSystem.Init();
		clawSystem.Init();
		liftSystem.Init();

		// Sensors
		distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
		distanceSensor.getDistance(DistanceUnit.CM); // test reading
		if (distanceSensor.didTimeoutOccur()) {
			faultyDistanceSensor = true;
			telemetry.addLine("[ERROR] Distance sensor is faulty!");
			telemetry.update();
		}

		// Camera and detection
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		detectionPipeline = new TeamPropDetectionPipeline(currentAlliance, Globals.IsDebugging());
		camera.setPipeline(detectionPipeline);
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				camera.startStreaming(Constants.Camera.CAMERA_WIDTH, Constants.Camera.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode)
			{
				telemetry.addData("Camera Error", errorCode);
				telemetry.update();
			}
		});
	}

	@Override
	protected void OnRun()
	{
		camera.closeCameraDevice();
		switch (currentPath) {
			case SHORT:
				RunShort();
				break;
			case LONG:
				RunLong();
				break;
		}
	}

	private void RunShort()
	{
		Pose2d purplePose = new Pose2d(0, 0, 0);
		Pose2d yellowPose = new Pose2d(0, 0, 0);
		Pose2d parkPose;
		double purpleTangent = 0;
		double yellowTangent = 0;
		double offset1 = -centimetersToInches(25);
		double offset2 = -centimetersToInches(10);
		Action yellowApproach;

		if (currentAlliance == Utilities.Alliance.RED) {
			parkPose = new Pose2d(centimetersToInches(parkingType == Utilities.ParkingPosition.CENTER ? 130 : 10), -centimetersToInches(75), -Math.PI / 2);
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(60), -centimetersToInches(10), 0);
				yellowPose = new Pose2d(centimetersToInches(65), -centimetersToInches(97), -Math.PI / 2);
				purpleTangent = 0;
				yellowTangent = Math.PI / 2;
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), 0, Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(85), -centimetersToInches(96), -Math.PI / 2);
				purpleTangent = 0;
				yellowTangent = Math.PI / 2;
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), -centimetersToInches(55), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(55), -centimetersToInches(98), -Math.PI / 2);
				purpleTangent = -Math.PI;
				yellowTangent = Math.PI / 2;
			}
		} else {
			offset1 *= -1;
			offset2 *= -1;
			parkPose = new Pose2d(centimetersToInches(parkingType == Utilities.ParkingPosition.CENTER ? 130 : 20), centimetersToInches(85), Math.PI / 2);
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(60), centimetersToInches(10), 0);
				yellowPose = new Pose2d(centimetersToInches(65), centimetersToInches(108), Math.PI / 2);
				purpleTangent = 0;
				yellowTangent = -Math.PI / 2;
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(80), centimetersToInches(63), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(50), centimetersToInches(108), Math.PI / 2);
				purpleTangent = 0;
				yellowTangent = -Math.PI / 2;
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), centimetersToInches(8), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(90), centimetersToInches(108), Math.PI / 2);
				purpleTangent = Math.PI;
				yellowTangent = -Math.PI / 2;
			}
		}

		// Add fallbacks for faulty distance sensor
		if (!faultyDistanceSensor)
			yellowApproach = ApproachWithDistSensor(mecanumDrive, distanceSensor, Constants.getBackdropDistance());
		else
			yellowApproach = mecanumDrive.actionBuilder(new Pose2d(yellowPose.position.x, yellowPose.position.y - offset2, yellowPose.heading.real))
					.setTangent(yellowTangent)
					.lineToYLinearHeading(yellowPose.position.y, yellowPose.heading.toDouble())
					.build();

		Actions.runBlocking(
				RunSequentially(
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER), // Close claw
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(mecanumDrive.pose)
										.splineToSplineHeading(purplePose, purpleTangent)
										.build(), // Move to purple
								tumblerSystem.MoveToPosition(Constants.getTumblerSpikeMark()), // Move tumbler to spike mark
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.25) // Move rotator to busy
						),
						WaitForMovementStop(mecanumDrive), // Wait for movement to stop
						clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.2d, Utilities.DelayDirection.AFTER), // Open claw
						RunInParallel(
								RunSequentially(
										mecanumDrive.actionBuilder(purplePose)
												.setTangent(yellowTangent)
												.lineToY(yellowPose.position.y - offset1)
												.splineToLinearHeading(new Pose2d(yellowPose.position.x, yellowPose.position.y - offset2, yellowPose.heading.toDouble()), yellowTangent)
												.build(), // Get into position for yellow approach
										yellowApproach // Approach yellow
								),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.25), // Move rotator to idle
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.25), // Move tumbler to load
								intakeSystem.RunIntakeFor(1) // Run intake for 1 second to push yellow
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH), // Close claw
						RunInParallel(
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5), // Move rotator to busy
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8), // Move tumbler to backdrop
								liftSystem.MoveToPosition(Constants.getLiftLevel1())
						),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.25d, Utilities.DelayDirection.BOTH), // Open claw
						RunInParallel(
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.25),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								liftSystem.MoveToPosition(Constants.getLiftPickup()),
								mecanumDrive.actionBuilder(yellowPose)
										.setTangent(0)
										.lineToX(parkPose.position.x)
										.setTangent(Math.PI / 2)
										.lineToY(parkPose.position.y)
										.build() // Move to park
						)
				)
		);
	}

	public void RunLong()
	{
		Pose2d purplePose = new Pose2d(0, 0, 0);
		Pose2d backdropIntermediaryPose = new Pose2d(0, 0, 0);
		Pose2d yellowPose = new Pose2d(0, 0, 0);
		Pose2d stackPose = new Pose2d(0, 0, 0);
		Pose2d backdropPose = new Pose2d(0, 0, 0);
		double safeDistance = -centimetersToInches(30);
		Action yellowApproach, backdropApproach;
		if (currentAlliance == Utilities.Alliance.RED) {
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(90), centimetersToInches(30), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(130), -centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(70), -centimetersToInches(223), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(131), centimetersToInches(54), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(85), -centimetersToInches(223), -Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(68), centimetersToInches(53), -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(130), -centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(85), -centimetersToInches(223), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(131), centimetersToInches(54), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(70), -centimetersToInches(223), -Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), 0, -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(130), -centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(55), -centimetersToInches(223), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(131), centimetersToInches(54), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(85), -centimetersToInches(223), -Math.PI / 2);
			}
		} else {
			safeDistance *= -1;
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(127), 0, -Math.PI);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(130), centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(70), centimetersToInches(223), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(133), -centimetersToInches(49), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(85), centimetersToInches(223), -Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), 0, -Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(130), centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(55), centimetersToInches(223), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(133), -centimetersToInches(49), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(85), centimetersToInches(223), -Math.PI / 2);
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), centimetersToInches(11), Math.PI / 2);
				backdropIntermediaryPose = new Pose2d(centimetersToInches(130), centimetersToInches(140), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(85), centimetersToInches(223), -Math.PI / 2);
				stackPose = new Pose2d(centimetersToInches(133), -centimetersToInches(49), -Math.PI / 2);
				backdropPose = new Pose2d(centimetersToInches(70), centimetersToInches(223), -Math.PI / 2);
			}
		}

		// Add fallbacks for faulty distance sensor
		if (!faultyDistanceSensor) {
			yellowApproach = ApproachWithDistSensor(mecanumDrive, distanceSensor, Constants.getBackdropDistance());
			backdropApproach = ApproachWithDistSensor(mecanumDrive, distanceSensor, Constants.getBackdropDistance());
		} else {
			yellowApproach = mecanumDrive.actionBuilder(new Pose2d(yellowPose.position.x, yellowPose.position.y - safeDistance, yellowPose.heading.real))
					.setTangent(yellowPose.heading.toDouble())
					.lineToYLinearHeading(yellowPose.position.y, yellowPose.heading.toDouble())
					.build();
			backdropApproach = mecanumDrive.actionBuilder(new Pose2d(backdropPose.position.x, backdropPose.position.y - safeDistance, backdropPose.heading.real))
					.setTangent(backdropPose.heading.toDouble())
					.lineToYLinearHeading(backdropPose.position.y, backdropPose.heading.toDouble())
					.build();
		}

		Actions.runBlocking(RunSequentially(
				WaitFor(longDelay),
				clawSystem.MoveToPosition(Constants.getClawBusy()),
				// Place purple
				RunInParallel(
						RunSequentially(
								mecanumDrive.actionBuilder(mecanumDrive.pose)
										.splineToLinearHeading(new Pose2d(purplePose.position.x, purplePose.position.y / 2, purplePose.heading.toDouble()), purplePose.heading.toDouble() / 2)
										.build(),
								mecanumDrive.actionBuilder(new Pose2d(purplePose.position.x, purplePose.position.y / 2, purplePose.heading.toDouble()))
										.lineToYConstantHeading(purplePose.position.y)
										.build()
						),
						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
						rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
				),
				RunInParallel(
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						intakeSystem.RunIntakeWithAntennaFor(0.3)
				),
				// Drive to intermediate backdrop position
				RunInParallel(
						mecanumDrive.actionBuilder(purplePose)
								.setTangent(0)
								.splineToConstantHeading(new Vector2d(backdropIntermediaryPose.position.x, 0), -backdropIntermediaryPose.heading.toDouble())
								.lineToYConstantHeading(backdropIntermediaryPose.position.y)
								.splineToConstantHeading(new Vector2d(yellowPose.position.x, yellowPose.position.y - safeDistance), -yellowPose.heading.toDouble())
								.build(),
						RunSequentially(
								RunInParallel(
										intakeSystem.RunIntakeFor(1),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.3),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5)
								),
								clawSystem.MoveToPosition(Constants.getClawBusy())
						)
				),
				// Drive to backdrop
				RunInParallel(
						yellowApproach,
						liftSystem.MoveToPositionWithDelay(Constants.getLiftLevel2(), 0.3),
						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.3),
						rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5)
				),
				WaitForMovementStop(mecanumDrive),
				// Drop pixel
				RunInParallel(
						RunSequentially(
								clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.BOTH),
								RunInParallel(
										liftSystem.MoveToPositionWithDelay(Constants.getSuspenderIdle(), 0.25),
										tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
										rotatorSystem.MoveToPosition(Constants.getRotatorIdle())
								)
						),
						intakeSystem.RunIntakeFor(0.75)
				),
				// Pickup stack pixel and drop
				RunInParallel(
						mecanumDrive.actionBuilder(yellowPose)
								.setTangent(0)
								.lineToXConstantHeading(backdropPose.position.x)
								.build(),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.4, Utilities.DelayDirection.AFTER)
				),
				RunInParallel(
						liftSystem.MoveToPosition(Constants.getLiftLevel2()),
						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.1),
						rotatorSystem.MoveToPosition(Constants.getRotatorBusy())
				),
				clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.BOTH),
				// Drive to stack
				RunInParallel(
						mecanumDrive.actionBuilder(backdropPose)
								.setTangent(-backdropPose.heading.toDouble())
								.splineToConstantHeading(new Vector2d(backdropIntermediaryPose.position.x, backdropIntermediaryPose.position.y), -backdropPose.heading.toDouble())
								.lineToYConstantHeading(stackPose.position.y)
								.build(),
						tumblerSystem.MoveToPosition(Constants.getTumblerIdle()),
						rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
						liftSystem.MoveToPositionWithDelay(Constants.getLiftPickup(), 0.3)
				),
				// Take 2 pixels
				WaitForMovementStop(mecanumDrive),
				intakeSystem.RunIntakeWithAntennaFor(0.3),
				intakeSystem.RunIntakeFor(0.2),
				intakeSystem.RunIntakeWithAntennaFor(0.3),
				// Drive to intermediate backdrop position
				RunInParallel(
						mecanumDrive.actionBuilder(stackPose)
								.setTangent(0)
								.splineToConstantHeading(new Vector2d(backdropIntermediaryPose.position.x, 0), -backdropIntermediaryPose.heading.toDouble())
								.lineToYConstantHeading(backdropIntermediaryPose.position.y)
								.splineToConstantHeading(new Vector2d(backdropPose.position.x, backdropPose.position.y - safeDistance), -backdropPose.heading.toDouble())
								.build(),
						intakeSystem.RunIntakeFor(0.6),
						RunSequentially(
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
								clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.4, Utilities.DelayDirection.AFTER)
						)
				),
				// Drive to backdrop
				RunInParallel(
						backdropApproach,
						liftSystem.MoveToPosition(Constants.getLiftLevel2()),
						tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
						rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.2)
				),
				WaitForMovementStop(mecanumDrive),
				// Drop first stack pixel
				clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.4, Utilities.DelayDirection.AFTER),
				RunInParallel(
						intakeSystem.RunIntakeFor(0.75),
						liftSystem.MoveToPositionWithDelay(Constants.getSuspenderIdle(), 0.25),
						tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
						rotatorSystem.MoveToPosition(Constants.getRotatorIdle())
				),
				// Pickup second stack pixel and drop
				clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.4, Utilities.DelayDirection.AFTER),
				RunInParallel(
						liftSystem.MoveToPosition(Constants.getLiftLevel2()),
						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.1),
						rotatorSystem.MoveToPosition(Constants.getRotatorBusy())
				),
				// Reset positions and park
				clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.1),
				RunInParallel(
						liftSystem.MoveToPositionWithDelay(Constants.getSuspenderIdle(), 0.2),
						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.2),
						rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.3)
				),
				WaitFor(10)
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
		if (currentPath == Utilities.PathType.SHORT)
			telemetry.addLine("> Make sure to place robot ~4 fingers away from the right hand side of the start tile!");
		else if (currentPath == Utilities.PathType.LONG)
			telemetry.addLine("> Make sure to place robot ~4 fingers away from the left hand side of the start tile!");
		telemetry.addLine("> Press play to start...");

		telemetry.update();
	}

	private void ConfigureAutonomous()
	{
		telemetry.setMsTransmissionInterval(25);

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
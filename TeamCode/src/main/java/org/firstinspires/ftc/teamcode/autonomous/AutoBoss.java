package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
import static org.firstinspires.ftc.teamcode.Utilities.WaitForMovementStop;
import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Boss", group = "Auto")
public class AutoBoss extends LinearOpMode
{
	private MecanumDrive mecanumDrive;
	private TumblerSystem tumblerSystem;
	private RotatorSystem rotatorSystem;
	private ClawSystem clawSystem;
	private IntakeSystem intakeSystem;
	private LiftSystem liftSystem;

	private OpenCvCamera camera;
	private TeamPropDetectionPipeline detectionPipeline;

	private Utilities.Alliance currentAlliance = null;
	private Utilities.PathType currentPath = null;
	private Utilities.ParkingPosition parkingType = null;
	private double longDelay = -1;

	@Override
	public void runOpMode()
	{
		ConfigureAutonomous();
		Init();
		Detection();
		Run();
	}

	private void Init()
	{
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2);
		Constants.Init();

		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), Globals.GetCurrentRobotType());

		tumblerSystem = new TumblerSystem(hardwareMap.get(DcMotorEx.class, "tumbler"));
		intakeSystem = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"), hardwareMap.get(Servo.class, "antenna"));
		rotatorSystem = new RotatorSystem(hardwareMap.get(Servo.class, "rotator"));
		clawSystem = new ClawSystem(hardwareMap.get(Servo.class, "claw"));
		liftSystem = new LiftSystem(hardwareMap.get(DcMotorEx.class, "lift1"), hardwareMap.get(DcMotorEx.class, "lift2"));

		tumblerSystem.Init();
		intakeSystem.Init();
		rotatorSystem.Init();
		clawSystem.Init();
		liftSystem.Init();

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

	private void Run()
	{
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
		Pose2d parkPose = new Pose2d(0, 0, 0);
		double purpleTangent = 0;
		double yellowTangent = 0;
		double offset1 = -centimetersToInches(25);
		double offset2 = -centimetersToInches(10);

		Utilities.DetectionCase detectionCase = detectionPipeline.getDetectionCase();

		if (currentAlliance == Utilities.Alliance.RED) {
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(60), -centimetersToInches(10), 0);
				yellowPose = new Pose2d(centimetersToInches(65), -centimetersToInches(97), -Math.PI / 2);
				parkPose = new Pose2d(centimetersToInches(parkingType == Utilities.ParkingPosition.LEFT ? 130 : 20), -centimetersToInches(97), -Math.PI / 2);
				purpleTangent = 0;
				yellowTangent = Math.PI / 2;
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), 0, Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(85), -centimetersToInches(96), -Math.PI / 2);
				parkPose = new Pose2d(centimetersToInches(parkingType == Utilities.ParkingPosition.LEFT ? 130 : 20), -centimetersToInches(96), -Math.PI / 2);
				purpleTangent = 0;
				yellowTangent = Math.PI / 2;
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), -centimetersToInches(55), Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(55), -centimetersToInches(98), -Math.PI / 2);
				parkPose = new Pose2d(centimetersToInches(parkingType == Utilities.ParkingPosition.LEFT ? 130 : 20), -centimetersToInches(96), -Math.PI / 2);
				purpleTangent = -Math.PI;
				yellowTangent = Math.PI / 2;
			}
		} else {
			offset1 *= -1;
			offset2 *= -1;
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(60), centimetersToInches(10), 0);
				yellowPose = new Pose2d(centimetersToInches(65), centimetersToInches(108), Math.PI / 2);
				parkPose = new Pose2d(centimetersToInches(parkingType == Utilities.ParkingPosition.LEFT ? 130 : 20), centimetersToInches(108), Math.PI / 2);
				purpleTangent = 0;
				yellowTangent = -Math.PI / 2;
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(75), centimetersToInches(63), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(55), centimetersToInches(108), Math.PI / 2);
				parkPose = new Pose2d(centimetersToInches(parkingType == Utilities.ParkingPosition.LEFT ? 130 : 20), centimetersToInches(108), Math.PI / 2);
				purpleTangent = 0;
				yellowTangent = -Math.PI / 2;
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), centimetersToInches(8), -Math.PI / 2);
				yellowPose = new Pose2d(centimetersToInches(90), centimetersToInches(108), Math.PI / 2);
				parkPose = new Pose2d(centimetersToInches(parkingType == Utilities.ParkingPosition.LEFT ? 130 : 20), centimetersToInches(108), Math.PI / 2);
				purpleTangent = Math.PI;
				yellowTangent = -Math.PI / 2;
			}
		}

		Actions.runBlocking( // Run the autonomous
				RunSequentially(
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER), // Close claw
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
								mecanumDrive.actionBuilder(purplePose)
										.setTangent(yellowTangent)
										.lineToY(yellowPose.position.y - offset1)
										.splineToLinearHeading(new Pose2d(yellowPose.position.x, yellowPose.position.y - offset2, yellowPose.heading.toDouble()), yellowTangent)
										.lineToYLinearHeading(yellowPose.position.y, yellowPose.heading.toDouble())
										.build(), // Move to yellow
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
										.lineToY(parkPose.position.y + 5)
										.build() // Move to park
						)
				)
		);
	}

	public void RunLong()
	{
		/*
		Pose2d purplePose = new Pose2d(0, 0, 0);
		Pose2d intermediaryPose = new Pose2d(centimetersToInches(130), centimetersToInches(5), -Math.PI / 2);
		double purpleTangent = 0;

		Utilities.DetectionCase detectionCase = detectionPipeline.getDetectionCase();

		if (currentAlliance == Utilities.Alliance.RED) {
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(127), 0, -Math.PI);
				purpleTangent = -Math.PI / 2;
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(117), centimetersToInches(15), -Math.PI);
				purpleTangent = -Math.PI / 2;
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(70), centimetersToInches(5), -Math.PI / 2);
				purpleTangent = -Math.PI / 2;
			}
		} else {
			if (detectionCase == Utilities.DetectionCase.CENTER) { // Center case is the same for both paths
				purplePose = new Pose2d(centimetersToInches(60), centimetersToInches(10), 0);
				purpleTangent = 0;
			} else if (detectionCase == Utilities.DetectionCase.LEFT) { // Left blue case is the same as right red case and vice versa
				purplePose = new Pose2d(centimetersToInches(110), centimetersToInches(10), -Math.PI / 2);
				purpleTangent = 0;
			} else if (detectionCase == Utilities.DetectionCase.RIGHT) { // Right blue case is the same as left red case and vice versa
				purplePose = new Pose2d(centimetersToInches(110), centimetersToInches(10), -Math.PI / 2);
				purpleTangent = Math.PI;
			}
		}

		Actions.runBlocking( // Run the autonomous
				RunSequentially(
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER), // Close claw
						mecanumDrive.actionBuilder(mecanumDrive.pose)
								.setTangent(purpleTangent)
								.lineToY(purplePose.position.y)
								.build()
						)
		);
		Actions.runBlocking( // Run the autonomous
				RunSequentially(
						RunInParallel(
								mecanumDrive.actionBuilder(mecanumDrive.pose)
										.lineToXSplineHeading(purplePose.position.x, purplePose.heading.toDouble())
										.build(), // Move to purple
								tumblerSystem.MoveToPosition(Constants.getTumblerSpikeMark()), // Move tumbler to spike mark
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.25) // Move rotator to busy
						),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.2d, Utilities.DelayDirection.AFTER), // Open claw
						RunInParallel(
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()), // Move rotator to busy
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()) // Move tumbler to backdrop
						)
				)
		);
		Actions.runBlocking( // Run the autonomous
				RunSequentially(
						mecanumDrive.actionBuilder(mecanumDrive.pose)
								.splineToLinearHeading(intermediaryPose, Math.PI/2)
								.build()
//						WaitFor(4),
//						mecanumDrive.actionBuilder(stackPose)
//								.lineToYLinearHeading(-centimetersToInches(150), -Math.PI / 2)
//								.build()
//						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH), // Close claw
//						RunInParallel(
//								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5), // Move rotator to busy
//								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8) // Move tumbler to backdrop
//						),
//						WaitForMovementStop(mecanumDrive),
//						clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.25d, Utilities.DelayDirection.BOTH), // Open claw
//						RunInParallel(
//								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.25),
//								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
//								mecanumDrive.actionBuilder(yellowPose)
//										.setTangent(0)
//										.lineToX(parkPose.position.x)
//										.build() // Move to park
//						)
				)
		);//*/

		if (currentAlliance == Utilities.Alliance.RED)
		{
			switch (detectionPipeline.getDetectionCase()) {
				case RIGHT:
					Actions.runBlocking(RunSequentially(
							clawSystem.MoveToPosition(Constants.getClawBusy()),
							WaitFor(longDelay),
							// Place purple
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
											.splineToSplineHeading(new Pose2d(centimetersToInches(65), centimetersToInches(2), -Math.PI / 2), 0)
											.build(),
									tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
							),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.2),

							// Place yellow
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(65), centimetersToInches(2), -Math.PI / 2))
											.setTangent(0)
											.lineToX(centimetersToInches(125))
											.setTangent(Math.PI / 2)
											.lineToY(-centimetersToInches(160))
											.splineToLinearHeading(new Pose2d(centimetersToInches(40), -centimetersToInches(225), -Math.PI / 2), Math.PI / 2)
											.build(),
									intakeSystem.RunIntakeFor(1),
									rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
							),
							clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
							RunInParallel(
									liftSystem.MoveToPosition(Constants.getLiftLevel2()),
									tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
							),
							WaitFor(0.2),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.5),

							// Reset positions and Park
							RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(47), centimetersToInches(225), -Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(200)), -Math.PI / 2)
//										.build(),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
									clawSystem.MoveToPosition(Constants.getClawIdle()),
									liftSystem.MoveToPosition(Constants.getLiftPickup())
							)
					));
					break;
				// =========================================================================================================================================
				case CENTER:
					Actions.runBlocking(RunSequentially(
							clawSystem.MoveToPosition(Constants.getClawBusy()),
							WaitFor(longDelay),
							// Place purple
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
											.lineToXSplineHeading(centimetersToInches(130), -Math.PI)
											.build(),
									tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
							),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.2),

							// Place yellow
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(130), 0, Math.PI))
											.splineToSplineHeading(new Pose2d(centimetersToInches(130), centimetersToInches(160), -Math.PI / 2), Math.PI / 2)
											.splineToLinearHeading(new Pose2d(centimetersToInches(53), centimetersToInches(224), -Math.PI / 2), Math.PI / 2)
											.build(),
									intakeSystem.RunIntakeFor(1),
									rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
							),
							clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
							RunInParallel(
									liftSystem.MoveToPosition(Constants.getLiftLevel2()),
									tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
							),
							WaitFor(0.2),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.5),

							// Reset positions and Park
							RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(47), centimetersToInches(224), -Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(200)), -Math.PI / 2)
//										.build(),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
									clawSystem.MoveToPosition(Constants.getClawIdle()),
									liftSystem.MoveToPosition(Constants.getLiftPickup())
							)
					));
					break;
				// =========================================================================================================================================
				case LEFT:
					Actions.runBlocking(RunSequentially(
							clawSystem.MoveToPosition(Constants.getClawBusy()),
							WaitFor(longDelay),
							// Place purple
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
											.splineToLinearHeading(new Pose2d(centimetersToInches(69), -centimetersToInches(5), Math.PI / 2), 0)
											.build(),
									tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
							),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.2),

							// Place yellow
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(69), -centimetersToInches(5), Math.PI / 2))
											.setTangent(0)
											.lineToX(centimetersToInches(135))
											.turnTo(-Math.PI / 2)
											.lineToY(-centimetersToInches(160))
											.setTangent(-Math.PI / 2)
											.splineToSplineHeading(new Pose2d(centimetersToInches(77), -centimetersToInches(212), -Math.PI / 2), -Math.PI / 2)
											.build(),
									intakeSystem.RunIntakeFor(1),
									rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
							),
							clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
							RunInParallel(
									liftSystem.MoveToPosition(Constants.getLiftLevel2()),
									tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
							),
							WaitFor(0.2),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.5),

							// Reset positions and Park
							RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(95), centimetersToInches(212), -Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(200)), -Math.PI / 2)
//										.build(),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
									clawSystem.MoveToPosition(Constants.getClawIdle()),
									liftSystem.MoveToPosition(Constants.getLiftPickup())
							)
					));
					break;

				case NONE:
					telemetry.addLine("Invalid case! (NONE)");
					telemetry.update();
					break;
			}
		}
		else if (currentAlliance == Utilities.Alliance.BLUE)
		{
			switch (detectionPipeline.getDetectionCase()) {
				case LEFT:
					Actions.runBlocking(RunSequentially(
							clawSystem.MoveToPosition(Constants.getClawBusy()),
							WaitFor(longDelay),
							// Place purple
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
											.splineToSplineHeading(new Pose2d(centimetersToInches(65), -centimetersToInches(5), Math.PI / 2), 0)
											.build(),
									tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
							),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.2),

							// Place yellow
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(65), centimetersToInches(2), Math.PI / 2))
											.setTangent(0)
											.lineToX(centimetersToInches(125))
											.setTangent(-Math.PI / 2)
											.lineToY(-centimetersToInches(160))
											.splineToLinearHeading(new Pose2d(centimetersToInches(33), -centimetersToInches(227), Math.PI / 2), -Math.PI / 2)
											.build(),
									intakeSystem.RunIntakeFor(1),
									rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
							),
							clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
							RunInParallel(
									liftSystem.MoveToPosition(Constants.getLiftLevel2()),
									tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
							),
							WaitFor(0.2),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.5),

							// Reset positions and Park
							RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(33), -centimetersToInches(224), Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), -centimetersToInches(200)), Math.PI / 2)
//										.build(),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
									clawSystem.MoveToPosition(Constants.getClawIdle()),
									liftSystem.MoveToPosition(Constants.getLiftPickup())
							)
					));
					break;
				// =========================================================================================================================================
				case CENTER:
					Actions.runBlocking(RunSequentially(
							clawSystem.MoveToPosition(Constants.getClawBusy()),
							WaitFor(longDelay),
							// Place purple
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
											.lineToXSplineHeading(centimetersToInches(130), Math.PI)
											.build(),
									tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
							),
							WaitFor(0.2),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.2),

							// Place yellow
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(130), 0, Math.PI))
											.splineToSplineHeading(new Pose2d(centimetersToInches(130), -centimetersToInches(160), Math.PI / 2), -Math.PI / 2)
											.splineToLinearHeading(new Pose2d(centimetersToInches(43), -centimetersToInches(227), Math.PI / 2), -Math.PI / 2)
											.build(),
									intakeSystem.RunIntakeFor(1),
									rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
							),
							clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
							RunInParallel(
									liftSystem.MoveToPosition(Constants.getLiftLevel2()),
									tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
							),
							WaitFor(0.2),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.5),

							// Reset positions and Park
							RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(40), -centimetersToInches(224), Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), -centimetersToInches(200)), Math.PI / 2)
//										.build(),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
									clawSystem.MoveToPosition(Constants.getClawIdle()),
									liftSystem.MoveToPosition(Constants.getLiftPickup())
							)
					));
					break;
				// =========================================================================================================================================
				case RIGHT:
					Actions.runBlocking(RunSequentially(
							clawSystem.MoveToPosition(Constants.getClawBusy()),
							WaitFor(longDelay),
							// Place purple
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
											.splineToSplineHeading(new Pose2d(centimetersToInches(65), centimetersToInches(5), -Math.PI / 2), 0)
											.build(),
									tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
							),
							WaitFor(0.2),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.2),

							// Place yellow
							RunInParallel(
									mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(70), centimetersToInches(5), -Math.PI / 2))
											.setTangent(0)
											.lineToX(centimetersToInches(130))
											.setTangent(Math.PI / 2)
											.lineToYLinearHeading(-centimetersToInches(150), Math.PI / 2)
											.splineToLinearHeading(new Pose2d(centimetersToInches(83), -centimetersToInches(218), Math.PI / 2), Math.PI / 2)
											.build(),
									intakeSystem.RunIntakeFor(1),
									rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
							),
							clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
							RunInParallel(
									liftSystem.MoveToPosition(Constants.getLiftLevel2()),
									tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
							),
							WaitFor(0.2),
							WaitForMovementStop(mecanumDrive),
							clawSystem.MoveToPosition(Constants.getClawIdle()),
							WaitFor(0.5),

							// Reset positions and Park
							RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(92), -centimetersToInches(215), Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), -centimetersToInches(200)), Math.PI / 2)
//										.build(),
									rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
									tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
									clawSystem.MoveToPosition(Constants.getClawIdle()),
									liftSystem.MoveToPosition(Constants.getLiftPickup())
							)
					));
					break;

				case NONE:
					telemetry.addLine("Invalid case! (NONE)");
					telemetry.update();
					break;
			}
		}
	}

	public void RunRedLong()
	{
		switch (detectionPipeline.getDetectionCase()) {
			case RIGHT:
				Actions.runBlocking(RunSequentially(
						clawSystem.MoveToPosition(Constants.getClawBusy()),
						WaitFor(longDelay),
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.splineToSplineHeading(new Pose2d(-centimetersToInches(65), -centimetersToInches(5), -Math.PI / 2), 0)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.2),

						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(65), -centimetersToInches(5), -Math.PI / 2))
										.setTangent(0)
										.lineToX(-centimetersToInches(125))
										.setTangent(Math.PI / 2)
										.lineToY(centimetersToInches(160))
										.splineToLinearHeading(new Pose2d(-centimetersToInches(40), centimetersToInches(225), -Math.PI / 2), Math.PI / 2)
										.build(),
								intakeSystem.RunIntakeFor(1),
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
						RunInParallel(
								liftSystem.MoveToPosition(Constants.getLiftLevel2()),
								tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
						),
						WaitFor(0.2),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.5),

						// Reset positions and Park
						RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(47), centimetersToInches(225), -Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(200)), -Math.PI / 2)
//										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								liftSystem.MoveToPosition(Constants.getLiftPickup())
						)
				));
				break;
			// =========================================================================================================================================
			case CENTER:
				Actions.runBlocking(RunSequentially(
						clawSystem.MoveToPosition(Constants.getClawBusy()),
						WaitFor(longDelay),
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.lineToXSplineHeading(-centimetersToInches(130), -Math.PI)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.2),

						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(130), 0, Math.PI))
										.splineToSplineHeading(new Pose2d(-centimetersToInches(130), centimetersToInches(160), -Math.PI / 2), Math.PI / 2)
										.splineToLinearHeading(new Pose2d(-centimetersToInches(53), centimetersToInches(224), -Math.PI / 2), Math.PI / 2)
										.build(),
								intakeSystem.RunIntakeFor(1),
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
						RunInParallel(
								liftSystem.MoveToPosition(Constants.getLiftLevel2()),
								tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
						),
						WaitFor(0.2),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.5),

						// Reset positions and Park
						RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(47), centimetersToInches(224), -Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(200)), -Math.PI / 2)
//										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								liftSystem.MoveToPosition(Constants.getLiftPickup())
						)
				));
				break;
			// =========================================================================================================================================
			case LEFT:
				Actions.runBlocking(RunSequentially(
						clawSystem.MoveToPosition(Constants.getClawBusy()),
						WaitFor(longDelay),
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.splineToSplineHeading(new Pose2d(-centimetersToInches(65), centimetersToInches(5), Math.PI / 2), 0)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.2),

						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(65), -centimetersToInches(5), Math.PI / 2))
										.setTangent(0)
										.lineToX(-centimetersToInches(135))
										.setTangent(Math.PI / 2)
										.lineToYLinearHeading(centimetersToInches(160), -Math.PI / 2)
										.splineToLinearHeading(new Pose2d(-centimetersToInches(90), centimetersToInches(215), -Math.PI / 2), -Math.PI / 2)
										.build(),
								intakeSystem.RunIntakeFor(1),
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
						RunInParallel(
								liftSystem.MoveToPosition(Constants.getLiftLevel2()),
								tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
						),
						WaitFor(0.2),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.5),

						// Reset positions and Park
						RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(95), centimetersToInches(212), -Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(200)), -Math.PI / 2)
//										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								liftSystem.MoveToPosition(Constants.getLiftPickup())
						)
				));
				break;

			case NONE:
				telemetry.addLine("Invalid case! (NONE)");
				telemetry.update();
				break;
		}
	}


	public void RunBlueLong()
	{
		switch (detectionPipeline.getDetectionCase()) {
			case LEFT:
				Actions.runBlocking(RunSequentially(
						clawSystem.MoveToPosition(Constants.getClawBusy()),
						WaitFor(longDelay),
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.splineToSplineHeading(new Pose2d(-centimetersToInches(65), centimetersToInches(2), Math.PI / 2), 0)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.2),

						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(65), centimetersToInches(2), Math.PI / 2))
										.setTangent(0)
										.lineToX(-centimetersToInches(125))
										.setTangent(-Math.PI / 2)
										.lineToY(-centimetersToInches(160))
										.splineToLinearHeading(new Pose2d(-centimetersToInches(33), -centimetersToInches(227), Math.PI / 2), -Math.PI / 2)
										.build(),
								intakeSystem.RunIntakeFor(1),
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
						RunInParallel(
								liftSystem.MoveToPosition(Constants.getLiftLevel2()),
								tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
						),
						WaitFor(0.2),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.5),

						// Reset positions and Park
						RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(33), -centimetersToInches(224), Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), -centimetersToInches(200)), Math.PI / 2)
//										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								liftSystem.MoveToPosition(Constants.getLiftPickup())
						)
				));
				break;
			// =========================================================================================================================================
			case CENTER:
				Actions.runBlocking(RunSequentially(
						clawSystem.MoveToPosition(Constants.getClawBusy()),
						WaitFor(longDelay),
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.lineToXSplineHeading(-centimetersToInches(130), Math.PI)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						WaitFor(0.2),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.2),

						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(130), 0, Math.PI))
										.splineToSplineHeading(new Pose2d(-centimetersToInches(130), -centimetersToInches(160), Math.PI / 2), -Math.PI / 2)
										.splineToLinearHeading(new Pose2d(-centimetersToInches(43), -centimetersToInches(227), Math.PI / 2), -Math.PI / 2)
										.build(),
								intakeSystem.RunIntakeFor(1),
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
						RunInParallel(
								liftSystem.MoveToPosition(Constants.getLiftLevel2()),
								tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
						),
						WaitFor(0.2),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.5),

						// Reset positions and Park
						RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(40), -centimetersToInches(224), Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), -centimetersToInches(200)), Math.PI / 2)
//										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								liftSystem.MoveToPosition(Constants.getLiftPickup())
						)
				));
				break;
			// =========================================================================================================================================
			case RIGHT:
				Actions.runBlocking(RunSequentially(
						clawSystem.MoveToPosition(Constants.getClawBusy()),
						WaitFor(longDelay),
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.splineToSplineHeading(new Pose2d(-centimetersToInches(65), -centimetersToInches(5), -Math.PI / 2), 0)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						WaitFor(0.2),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.2),

						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(70), centimetersToInches(5), -Math.PI / 2))
										.setTangent(0)
										.lineToX(-centimetersToInches(130))
										.setTangent(Math.PI / 2)
										.lineToYLinearHeading(-centimetersToInches(150), Math.PI / 2)
										.splineToLinearHeading(new Pose2d(-centimetersToInches(83), -centimetersToInches(218), Math.PI / 2), Math.PI / 2)
										.build(),
								intakeSystem.RunIntakeFor(1),
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad())
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.AFTER),
						RunInParallel(
								liftSystem.MoveToPosition(Constants.getLiftLevel2()),
								tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.7)
						),
						WaitFor(0.2),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.5),

						// Reset positions and Park
						RunInParallel(
//								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(92), -centimetersToInches(215), Math.PI / 2))
//										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), -centimetersToInches(200)), Math.PI / 2)
//										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.2),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								liftSystem.MoveToPosition(Constants.getLiftPickup())
						)
				));
				break;

			case NONE:
				telemetry.addLine("Invalid case! (NONE)");
				telemetry.update();
				break;
		}
	}

	private void Detection()
	{
		while (!isStarted()) {
			telemetry.addData("[INFO] Alliance", currentAlliance.name());
			telemetry.addData("[INFO] Path", currentPath.name());
			telemetry.addData("[INFO] Case", detectionPipeline.getDetectionCase().name());
			telemetry.addData("[INFO] Parking", parkingType.name());
			telemetry.update();
		}
	}

	private void ConfigureAutonomous()
	{
		telemetry.setMsTransmissionInterval(50);

		telemetry.addLine("[CONFIGURE] Please select an alliance");
		telemetry.addLine("[CONFIGURE] Press A for Red Alliance");
		telemetry.addLine("[CONFIGURE] Press B for Blue Alliance");
		telemetry.update();
		while (currentAlliance == null && !isStopRequested()) {
			if (gamepad1.a || gamepad2.a) currentAlliance = Utilities.Alliance.RED;
			else if (gamepad1.b || gamepad2.b) currentAlliance = Utilities.Alliance.BLUE;
		}

		telemetry.addLine("[CONFIGURE] Please select a path");
		telemetry.addLine("[CONFIGURE] Press X for Short Path");
		telemetry.addLine("[CONFIGURE] Press Y for Long Path");
		telemetry.update();
		while (currentPath == null && !isStopRequested()) {
			if (gamepad1.x || gamepad2.x) currentPath = Utilities.PathType.SHORT;
			else if (gamepad1.y || gamepad2.y) currentPath = Utilities.PathType.LONG;
		}

		telemetry.addLine("[CONFIGURE] Please select a parking strategy");
		telemetry.addLine("[CONFIGURE] Press A for Left Parking");
		telemetry.addLine("[CONFIGURE] Press B for Right Parking");
		telemetry.update();
		while (parkingType == null && !isStopRequested()) {
			if (gamepad1.a || gamepad2.a) parkingType = Utilities.ParkingPosition.LEFT;
			else if (gamepad1.b || gamepad2.b) parkingType = Utilities.ParkingPosition.RIGHT;
		}

		if (currentPath == Utilities.PathType.LONG) {
			telemetry.addLine("[CONFIGURE] Long path selected. Please select a delay");
			telemetry.addLine("[CONFIGURE] Press X for 0 second delay");
			telemetry.addLine("[CONFIGURE] Press Y for 5 second delay");
			telemetry.update();
			while (longDelay < 0 && !isStopRequested()) {
				if (gamepad1.x || gamepad2.x) longDelay = 0;
				else if (gamepad1.y || gamepad2.y) longDelay = 5;
			}
		}

		telemetry.clearAll();
		telemetry.update();
	}
}
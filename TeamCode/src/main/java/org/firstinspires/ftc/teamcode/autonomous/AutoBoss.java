package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.GetCurrentRobotType;
import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
import static org.firstinspires.ftc.teamcode.Utilities.WaitForMovementStop;
import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
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

@Autonomous(name = "AutoBoss", group = "Auto")
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

	@Override
	public void runOpMode()
	{
		SelectAllianceAndPath();
		Init();
		Detection();
		Run();
	}

	private void SelectAllianceAndPath()
	{
		telemetry.setMsTransmissionInterval(50);

		telemetry.addLine("Please select an alliance");
		telemetry.addLine("Press A for Red Alliance");
		telemetry.addLine("Press B for Blue Alliance");
		telemetry.update();
		while (currentAlliance == null && !isStopRequested()) {
			if (gamepad1.a || gamepad2.a) currentAlliance = Utilities.Alliance.RED;
			else if (gamepad1.b || gamepad2.b) currentAlliance = Utilities.Alliance.BLUE;
		}

		telemetry.addLine("Please select a path");
		telemetry.addLine("Press X for Short Path");
		telemetry.addLine("Press Y for Long Path");
		telemetry.update();
		while (currentPath == null && !isStopRequested()) {
			if (gamepad1.x || gamepad2.x) currentPath = Utilities.PathType.SHORT;
			else if (gamepad1.y || gamepad2.y) currentPath = Utilities.PathType.LONG;
		}

		telemetry.addData("Alliance: ", currentAlliance.name());
		telemetry.addData("Path: ", currentPath.name());
		telemetry.update();
	}

	private void Init()
	{
		// Robot systems
		Utilities.RobotType robotType = GetCurrentRobotType(hardwareMap, telemetry, gamepad1, gamepad2);
		Constants.Init(robotType);

		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), robotType);

		tumblerSystem = new TumblerSystem(hardwareMap.get(DcMotorEx.class, "tumbler"));
		intakeSystem = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"), hardwareMap.get(Servo.class, "antenna"));
		rotatorSystem = new RotatorSystem(hardwareMap.get(Servo.class, "rotator"));
		clawSystem = new ClawSystem(hardwareMap.get(Servo.class, "claw"));
		liftSystem = new LiftSystem(hardwareMap.get(DcMotorEx.class, "lift1"), hardwareMap.get(DcMotorEx.class, "lift2"));

		tumblerSystem.setRobotType(robotType);

		tumblerSystem.Init();
		intakeSystem.Init();
		rotatorSystem.Init();
		clawSystem.Init();
		liftSystem.Init();

		// Camera and detection
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		detectionPipeline = new TeamPropDetectionPipeline();
		detectionPipeline.setAlliance(currentAlliance);
		detectionPipeline.setDebug(Utilities.IsDebugging(hardwareMap));
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
			}
		});
	}

	private void Detection()
	{
		while (!isStarted()) {
			telemetry.addData("Alliance: ", currentAlliance.name());
			telemetry.addData("Path: ", currentPath.name());
			telemetry.addData("Case: ", detectionPipeline.getDetectionCase().name());
			telemetry.update();
		}
	}

	private void Run()
	{
		switch (currentAlliance) {
			case RED:
				switch (currentPath) {
					case SHORT:
						RunRedShort();
						break;
					case LONG:
						RunRedLong();
						break;
				}
				break;
			case BLUE:
				switch (currentPath) {
					case SHORT:
						RunBlueShort();
						break;
					case LONG:
						RunBlueLong();
						break;
				}
				break;
		}
	}


	private void RunRedShort()
	{
		Actions.runBlocking(clawSystem.MoveToPosition(Constants.getClawBusy()));
		switch (detectionPipeline.getDetectionCase()) {
			case LEFT:
				Actions.runBlocking(
						RunSequentially(
								// Place purple
								RunInParallel(
										mecanumDrive.actionBuilder(mecanumDrive.pose)
												.splineToSplineHeading(new Pose2d(centimetersToInches(70), 0, Math.PI / 2), 0)
												.build(),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
								),
								WaitForMovementStop(mecanumDrive),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								WaitFor(0.2),
								// Place yellow
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(70), 0, Math.PI / 2))
												.lineToY(-centimetersToInches(10))
												.splineToSplineHeading(new Pose2d(centimetersToInches(75), -centimetersToInches(100), -Math.PI / 2), Math.PI / 2)
												.build(),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
										intakeSystem.RunIntakeFor(1)
								),
								clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
								RunInParallel(
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.8),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 1)
								),
								WaitFor(0.2),
								WaitForMovementStop(mecanumDrive),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								WaitFor(0.2),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								WaitFor(0.5),
								// Reset positions and Park
								RunInParallel(
										rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
										mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(70), -centimetersToInches(95), -Math.PI / 2))
												.setTangent(0)
												.lineToX(centimetersToInches(120))
												.build()
								)
						));
				break;
			// =========================================================================================================================================
			case CENTER:
				Actions.runBlocking(RunSequentially(
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.splineToSplineHeading(new Pose2d(centimetersToInches(60), -centimetersToInches(10), 0), 0)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.2),

						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(60), -centimetersToInches(10), 0))
										.setTangent(-Math.PI / 2)
										.lineToY(-centimetersToInches(20))
										.splineToSplineHeading(new Pose2d(centimetersToInches(70), -centimetersToInches(86), -Math.PI / 2), Math.PI / 2)
										.lineToYSplineHeading(-centimetersToInches(96), -Math.PI / 2)
										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
								intakeSystem.RunIntakeFor(1)
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
						RunInParallel(
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8)
						),
						WaitFor(0.2),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.2),
						tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
						WaitFor(0.5),
						mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(70), -centimetersToInches(96), -Math.PI / 2))
								.setTangent(Math.PI / 2)
								.lineToYSplineHeading(centimetersToInches(177), -Math.PI / 2)
								.build(),
						telemetryPacket -> {
							mecanumDrive.updatePoseEstimate();
							telemetry.addData("heading", mecanumDrive.pose.heading);
							telemetry.addData("x", mecanumDrive.pose.position.x);
							telemetry.addData("y", mecanumDrive.pose.position.y);
							telemetry.update();
							return false;
						},
						intakeSystem.RunIntakeWithAntennaFor(1.5),
						WaitFor(1),
						intakeSystem.RunIntakeWithAntennaFor(1.5),
						mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(70), centimetersToInches(177), -Math.PI/2))
								.lineToYSplineHeading(-centimetersToInches(96), -Math.PI / 2)
								.build(),
						telemetryPacket -> {
							return true;
						}
						// Reset positions and Park
//						RunInParallel(
//								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
//								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
//								mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(60), -centimetersToInches(95), -Math.PI / 2))
//										.setTangent(0)
//										.lineToX(centimetersToInches(120))
//										.build()
//						)
				));
				break;
			// =========================================================================================================================================
			case RIGHT:
				Actions.runBlocking(RunSequentially(
								// Place purple
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
												.splineToLinearHeading(new Pose2d(centimetersToInches(70), -centimetersToInches(53), Math.PI / 2), Math.PI)
												.build(),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
								),
								WaitForMovementStop(mecanumDrive),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								WaitFor(0.2),

								// Place yellow
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(70), -centimetersToInches(53), Math.PI / 2))
												.splineToLinearHeading(new Pose2d(centimetersToInches(40), -centimetersToInches(100), -Math.PI / 2), Math.PI / 2)
												.build(),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
										intakeSystem.RunIntakeFor(1)
								),
								clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
								RunInParallel(
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8)
								),
								WaitFor(0.2),
								WaitForMovementStop(mecanumDrive),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								WaitFor(0.2),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								WaitFor(0.5),

								// Reset positions and Park
								RunInParallel(
										rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
										mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(40), -centimetersToInches(95), -Math.PI / 2))
												.setTangent(0)
												.lineToX(centimetersToInches(120))
												.build()
								)
						)
				);
				break;

			case NONE:
				telemetry.addLine("Invalid case! (NONE)");
				telemetry.update();
				break;
		}
	}


	private void RunBlueShort()
	{
		Actions.runBlocking(clawSystem.MoveToPosition(Constants.getClawBusy()));
		switch (detectionPipeline.getDetectionCase()) {
			case LEFT:
				Actions.runBlocking(
						RunSequentially(
								// Place purple
								RunInParallel(
										mecanumDrive.actionBuilder(mecanumDrive.pose)
												.splineToSplineHeading(new Pose2d(centimetersToInches(70), 0, -Math.PI / 2), 0)
												.build(),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
								),
								WaitForMovementStop(mecanumDrive),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								WaitFor(0.2),
								// Place yellow
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(70), 0, -Math.PI / 2))
												.lineToY(-centimetersToInches(10))
												.splineToSplineHeading(new Pose2d(centimetersToInches(75), centimetersToInches(100), Math.PI / 2), -Math.PI / 2)
												.build(),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
										intakeSystem.RunIntakeFor(1)
								),
								clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
								RunInParallel(
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.8),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 1)
								),
								WaitFor(0.2),
								WaitForMovementStop(mecanumDrive),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								WaitFor(0.2),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								WaitFor(0.5),
								// Reset positions and Park
								RunInParallel(
										rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
										mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(70), centimetersToInches(100), Math.PI / 2))
												.setTangent(0)
												.lineToX(centimetersToInches(120))
												.build()
								)
						));
				break;
			// =========================================================================================================================================
			case CENTER:
				Actions.runBlocking(RunSequentially(
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.splineToSplineHeading(new Pose2d(centimetersToInches(60), centimetersToInches(10), 0), 0)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.2),

						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(60), centimetersToInches(10), 0))
										.setTangent(-Math.PI / 2)
										.lineToY(-centimetersToInches(30))
										.splineToSplineHeading(new Pose2d(centimetersToInches(65), centimetersToInches(100), Math.PI / 2), -Math.PI / 2)
										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
								intakeSystem.RunIntakeFor(1)
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
						RunInParallel(
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8)
						),
						WaitFor(0.2),
						WaitForMovementStop(mecanumDrive),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						WaitFor(0.2),
						tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
						WaitFor(0.5),

						// Reset positions and Park
						RunInParallel(
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
								mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(65), centimetersToInches(100), Math.PI / 2))
										.setTangent(0)
										.lineToX(centimetersToInches(120))
										.build()
						)
				));
				break;
			// =========================================================================================================================================
			case RIGHT:
				Actions.runBlocking(RunSequentially(
								// Place purple
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
												.splineToLinearHeading(new Pose2d(centimetersToInches(70), centimetersToInches(53), -Math.PI / 2), -Math.PI)
												.build(),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
								),
								WaitForMovementStop(mecanumDrive),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								WaitFor(0.2),

								// Place yellow
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(70), centimetersToInches(53), -Math.PI / 2))
												.splineToLinearHeading(new Pose2d(centimetersToInches(40), centimetersToInches(100), Math.PI / 2), -Math.PI / 2)
												.build(),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
										intakeSystem.RunIntakeFor(1)
								),
								clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
								RunInParallel(
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8)
								),
								WaitFor(0.2),
								WaitForMovementStop(mecanumDrive),
								clawSystem.MoveToPosition(Constants.getClawIdle()),
								WaitFor(0.2),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								WaitFor(0.5),

								// Reset positions and Park
								RunInParallel(
										rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
										mecanumDrive.actionBuilder(new Pose2d(centimetersToInches(40), centimetersToInches(95), Math.PI / 2))
												.setTangent(0)
												.lineToX(centimetersToInches(120))
												.build()
								)
						)
				);
				break;

			case NONE:
				telemetry.addLine("Invalid case! (NONE)");
				telemetry.update();
				break;
		}
	}


	public void RunRedLong()
	{
		Actions.runBlocking(clawSystem.MoveToPosition(Constants.getClawBusy()));
		switch (detectionPipeline.getDetectionCase()) {
			case RIGHT:
				Actions.runBlocking(RunSequentially(
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
		Actions.runBlocking(clawSystem.MoveToPosition(Constants.getClawBusy()));
		switch (detectionPipeline.getDetectionCase()) {
			case LEFT:
				Actions.runBlocking(RunSequentially(
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
}